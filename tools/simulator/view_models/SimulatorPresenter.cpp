#include "tools/simulator/view_models/SimulatorPresenter.hpp"
#include <algorithm>
#include <cctype>
#include <format>

namespace simulator
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::theme::ButtonRole;

        constexpr std::array<ActionSpec, 5> operateActions{ {
            { OperateAction::bringUp, "Bring Up" },
            { OperateAction::enable, "Enable", ButtonRole::Start },
            { OperateAction::disable, "Disable", ButtonRole::Stop },
            { OperateAction::clearFaults, "Clear Faults", ButtonRole::Reset },
            { OperateAction::rearm, "Re-arm…" },
        } };

        constexpr std::array<ActionSpec, 2> moveActions{ {
            { MoveAction::move, "Move", ButtonRole::Primary },
            { MoveAction::abort, "Abort", ButtonRole::Stop },
        } };

        constexpr std::array<std::string_view, arm::jointCount> jointLabels{ "J1", "J2", "J3", "J4", "J5", "J6" };

        constexpr std::array<std::pair<Control, std::string_view>, 5> operateControls{ {
            { Control::bringUp, "Bring Up" },
            { Control::enable, "Enable" },
            { Control::disable, "Disable" },
            { Control::clearFaults, "Clear Faults" },
            { Control::rearm, "Re-arm" },
        } };

        constexpr std::array<std::pair<Control, std::string_view>, 2> moveControls{ {
            { Control::move, "Move" },
            { Control::abort, "Abort" },
        } };

        std::array<ui::model::FieldSpec, arm::jointCount> MoveFields(const arm::JointLimits& limits)
        {
            std::array<ui::model::FieldSpec, arm::jointCount> fields{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                fields[joint].id = ui::model::FieldId{ static_cast<uint16_t>(joint) };
                fields[joint].label = jointLabels[joint];
                fields[joint].suffix = "rad";
                fields[joint].number = { limits[joint].lower.Value(), limits[joint].upper.Value(), 0.01, 0.0, 3 };
            }
            return fields;
        }

        std::string SentenceCase(std::string_view text)
        {
            std::string sentence{ text };
            if (!sentence.empty())
                sentence.front() = static_cast<char>(std::toupper(static_cast<unsigned char>(sentence.front())));
            return sentence;
        }

        std::string TargetName(const arm_host::ArmTarget& target)
        {
            switch (target.kind)
            {
                case arm_host::TargetKind::simulated:
                    return "Simulated joints";
                case arm_host::TargetKind::virtualBus:
                    return "Virtual bus " + target.bus;
                case arm_host::TargetKind::real:
                    return "REAL ARM — " + target.bus;
            }
            return "Unknown target";
        }

        std::string WaitingForCalibration(const arm_host::ArmSnapshot& snapshot)
        {
            std::string text{ "Waiting for calibration:" };
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                const auto state = snapshot.readings[joint].state;
                if (state != can::FocMotorState::ready && state != can::FocMotorState::running)
                    text += std::format("{} {}", text.back() == ':' ? "" : ",", jointLabels[joint]);
            }
            return text;
        }

        std::string Misconfigured(const arm_host::ArmSnapshot& snapshot)
        {
            if (!snapshot.misconfiguration.has_value())
                return "Misconfigured";
            return std::format("Misconfigured: {} {}", jointLabels[snapshot.misconfiguration->joint],
                arm_host::MisconfigurationText(snapshot.misconfiguration->reason));
        }

        std::pair<std::string, ui::theme::StatusLevel> Stage(const arm_host::ArmSnapshot& snapshot)
        {
            using ui::theme::StatusLevel;
            using enum arm::ArmState;

            if (!snapshot.targetReady)
                return { "Disconnected", StatusLevel::Neutral };
            switch (snapshot.state)
            {
                case unpowered:
                    return { "Unpowered", StatusLevel::Neutral };
                case checking:
                    return { "Checking contracts…", StatusLevel::Neutral };
                case incompatible:
                    return { "Contracts incompatible", StatusLevel::Fault };
                case waitingForCalibration:
                    return { WaitingForCalibration(snapshot), StatusLevel::Warning };
                case configuring:
                    return { "Configuring motor controllers…", StatusLevel::Neutral };
                case misconfigured:
                    return { Misconfigured(snapshot), StatusLevel::Fault };
                case ready:
                    return { "Ready", StatusLevel::Neutral };
                case enabling:
                    return { "Enabling…", StatusLevel::Neutral };
                case enabled:
                    return { "Enabled", StatusLevel::Ok };
                case moving:
                    return { "Moving", StatusLevel::Ok };
                case disabling:
                    return { "Disabling…", StatusLevel::Neutral };
                case tripped:
                    return { "Tripped", StatusLevel::Fault };
            }
            return { "Unknown", StatusLevel::Neutral };
        }

        std::string ContractsText(can::ContractCompatibility contracts)
        {
            switch (contracts)
            {
                case can::ContractCompatibility::compatible:
                    return "Contracts: compatible";
                case can::ContractCompatibility::incompatible:
                    return "Contracts: incompatible";
                default:
                    return "Contracts: unchecked";
            }
        }
    }

    SimulatorPresenter::SimulatorPresenter(arm_host::ArmCommands& arm, ui::shell::ShellView& shell, ui::shell::FormView& operateView,
        ui::shell::FormView& moveView, const arm::JointLimits& limits, infra::TimePoint start)
        : arm(arm)
        , shell(shell)
        , operateView(operateView)
        , moveView(moveView)
        , limits(limits)
        , log(start)
    {
        forms.operateModel.onActionTriggered = [this](ui::model::ActionId action)
        {
            OnOperateAction(action);
        };
        forms.moveModel.onActionTriggered = [this](ui::model::ActionId action)
        {
            OnMoveAction(action);
        };

        operateView.Build(forms.operateModel);
        moveView.Build(forms.moveModel);
        Refresh(snapshot, {});
    }

    SimulatorPresenter::Forms::Forms(const arm::JointLimits& limits)
        : moveFields(MoveFields(limits))
        , operateSpec{ {}, {}, operateActions, {} }
        , moveSpec{ {}, moveFields, moveActions, {} }
        , operateModel(operateSpec, {}, {})
        , moveModel(moveSpec, moveValues, {})
    {}

    void SimulatorPresenter::Refresh(const arm_host::ArmSnapshot& newSnapshot, std::span<const arm_host::ArmEvent> events)
    {
        snapshot = newSnapshot;
        TrackSequence();
        for (const auto& event : events)
            Handle(event);

        rows = JointRows(snapshot, limits);
        scene.Update(snapshot);
        UpdateStatus();
        UpdateControls();
    }

    void SimulatorPresenter::EmergencyStop()
    {
        Post(arm_host::EmergencyStop{});
    }

    void SimulatorPresenter::SwitchTarget(const arm_host::ArmTarget& target)
    {
        const auto name = std::format("Switch to {}", TargetName(target));
        pending[arm.SwitchTarget(target)] = name;
        shell.SetStatus(std::format("{} requested…", name));
    }

    void SimulatorPresenter::ConfirmRearm()
    {
        Post(arm_host::Rearm{});
    }

    bool SimulatorPresenter::InjectFault(std::size_t joint)
    {
        if (snapshot.target.kind != arm_host::TargetKind::simulated || joint >= arm::jointCount)
            return false;

        Post(arm_host::InjectFault{ joint, can::FocFaultCode::overCurrent });
        return true;
    }

    bool SimulatorPresenter::SilenceJoint(std::size_t joint)
    {
        if (snapshot.target.kind != arm_host::TargetKind::simulated || joint >= arm::jointCount)
            return false;

        Post(arm_host::SilenceJoint{ joint });
        return true;
    }

    bool SimulatorPresenter::SelectJoint(int joint)
    {
        return scene.Select(joint);
    }

    bool SimulatorPresenter::SingleStep(Control control)
    {
        if (!State(control).enabled)
            return false;

        if (control == Control::checkContracts)
            Post(arm_host::CheckContracts{});
        else if (control == Control::positionMode)
            Post(arm_host::SelectMode{ can::FocMotorMode::position });
        else if (control == Control::start)
            Post(arm_host::StartJoints{});
        else if (control == Control::stop)
            Post(arm_host::StopJoints{});
        else
            return false;
        return true;
    }

    const ControlState& SimulatorPresenter::State(Control control) const
    {
        return states[static_cast<std::size_t>(control)];
    }

    const StatusStrip& SimulatorPresenter::Status() const
    {
        return status;
    }

    const std::array<JointRow, arm::jointCount>& SimulatorPresenter::Rows() const
    {
        return rows;
    }

    const ArmSceneModel& SimulatorPresenter::Scene() const
    {
        return scene;
    }

    const EventLog& SimulatorPresenter::Log() const
    {
        return log;
    }

    const std::string& SimulatorPresenter::OperateHint() const
    {
        return operateHint;
    }

    const std::string& SimulatorPresenter::MoveHint() const
    {
        return moveHint;
    }

    ui::model::FormModel& SimulatorPresenter::MoveForm()
    {
        return forms.moveModel;
    }

    void SimulatorPresenter::SetRearmPrompt(const ui::Callback<void(std::string_view prompt)>& prompt)
    {
        rearmPrompt = prompt;
    }

    void SimulatorPresenter::OnOperateAction(ui::model::ActionId action)
    {
        if (action == OperateAction::bringUp)
            Post(arm_host::BringUp{});
        else if (action == OperateAction::enable)
            Post(arm_host::Enable{});
        else if (action == OperateAction::disable)
            Post(arm_host::Disable{});
        else if (action == OperateAction::clearFaults)
            Post(arm_host::ClearFaults{});
        else if (action == OperateAction::rearm && rearmPrompt)
            rearmPrompt(std::format("Re-arm after the trip by {}?", tripSource.empty() ? std::string{ "an unknown source" } : tripSource));
    }

    void SimulatorPresenter::OnMoveAction(ui::model::ActionId action)
    {
        if (action == MoveAction::abort)
            return Post(arm_host::AbortMove{});

        if (action != MoveAction::move || !State(Control::move).enabled)
            return;

        arm::JointPositions target{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            target[joint] = foc::Radians{ forms.moveModel.Float(forms.moveFields[joint].id) };
        Post(arm_host::MoveTo{ target });
    }

    void SimulatorPresenter::Post(const arm_host::ArmCommand& command)
    {
        const auto name = SentenceCase(arm_host::CommandName(command));
        pending[arm.Post(command)] = name;
        shell.SetStatus(std::format("{} requested…", name));
    }

    void SimulatorPresenter::Handle(const arm_host::ArmEvent& event)
    {
        log.Add(event);

        if (event.kind == arm_host::ArmEventKind::tripped)
            tripSource = std::format("Joint {} ({})", event.joint + 1, event.fault == can::FocFaultCode::none ? std::string_view{ "silent" } : FaultName(event.fault));

        using enum arm_host::ArmEventKind;
        const auto request = pending.find(event.command);
        if (request == pending.end() || (event.kind != accepted && event.kind != refused && event.kind != targetReady && event.kind != targetFailed))
            return;

        if (event.kind == accepted)
            shell.SetStatus(std::format("{}: accepted", request->second));
        else if (event.kind == targetReady)
            shell.SetStatus(std::format("{}: done", request->second));
        else if (event.kind == targetFailed)
            shell.SetStatus(std::format("{} failed: {}", request->second, event.reason));
        else
            shell.SetStatus(std::format("{} refused: {}", request->second, event.reason));
        pending.erase(request);
    }

    void SimulatorPresenter::UpdateStatus()
    {
        status.target = TargetName(snapshot.target);
        status.realArm = snapshot.target.kind == arm_host::TargetKind::real;
        const bool stale = snapshot.targetReady && unchangedFrames >= staleFrames;
        if (stale)
        {
            status.connection = "No data from the arm";
            status.connectionLevel = ui::theme::StatusLevel::Warning;
        }
        else
        {
            status.connection = snapshot.targetReady ? "Connected" : "Disconnected";
            status.connectionLevel = snapshot.targetReady ? ui::theme::StatusLevel::Ok : ui::theme::StatusLevel::Neutral;
        }
        std::tie(status.stage, status.stageLevel) = Stage(snapshot);
        status.contracts = ContractsText(snapshot.contracts);
        const std::string_view source = tripSource.empty() ? std::string_view{ "unknown source" } : std::string_view{ tripSource };
        status.trip = snapshot.tripped ? std::format("Tripped — {}", source) : std::string{ "Trip: —" };
        status.tripLevel = snapshot.tripped ? ui::theme::StatusLevel::Fault : ui::theme::StatusLevel::Neutral;
        status.overruns = std::format("Overruns: {}", snapshot.overruns);
        status.overrunLevel = snapshot.overruns == 0 ? ui::theme::StatusLevel::Neutral : ui::theme::StatusLevel::Warning;
    }

    void SimulatorPresenter::UpdateControls()
    {
        const auto targetProblem = TargetProblem();
        for (std::size_t control = 0; control != states.size(); ++control)
            states[control] = Evaluate(static_cast<Control>(control), snapshot, targetProblem);

        for (const auto& action : operateActions)
            operateView.SetActionEnabled(action.id, State(operateControls[action.id.value].first).enabled);
        for (const auto& action : moveActions)
            moveView.SetActionEnabled(action.id, State(moveControls[action.id.value].first).enabled);

        operateHint = Hint(operateControls);
        moveHint = Hint(moveControls);
    }

    void SimulatorPresenter::TrackSequence()
    {
        if (snapshot.sequence == lastSequence)
        {
            ++unchangedFrames;
            return;
        }

        lastSequence = snapshot.sequence;
        unchangedFrames = 0;
    }

    std::string SimulatorPresenter::TargetProblem() const
    {
        const auto violation = forms.moveModel.Validate();
        if (!violation)
            return {};
        return std::format("{} outside its limits", jointLabels[violation->field.value]);
    }

    std::string SimulatorPresenter::Hint(std::span<const std::pair<Control, std::string_view>> controls) const
    {
        std::string hint;
        for (const auto& [control, label] : controls)
            if (!State(control).enabled)
                hint += std::format("{}{}: {}", hint.empty() ? "" : "\n", label, State(control).reason);
        return hint;
    }
}
