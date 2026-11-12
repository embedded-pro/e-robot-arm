#include "tools/simulator/view_models/SimulatorPresenter.hpp"
#include <cctype>
#include <format>

namespace simulator
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::theme::ButtonRole;

        constexpr std::array<ActionSpec, 6> operateActions{ {
            { OperateAction::checkContracts, "Check Contracts" },
            { OperateAction::positionMode, "Position Mode" },
            { OperateAction::start, "Start", ButtonRole::Start },
            { OperateAction::stop, "Stop", ButtonRole::Stop },
            { OperateAction::clearFaults, "Clear Faults", ButtonRole::Reset },
            { OperateAction::rearm, "Re-arm…" },
        } };

        constexpr std::array<ActionSpec, 2> moveActions{ {
            { MoveAction::move, "Move", ButtonRole::Primary },
            { MoveAction::abort, "Abort", ButtonRole::Stop },
        } };

        constexpr std::array<std::string_view, arm::jointCount> jointLabels{ "J1", "J2", "J3", "J4", "J5", "J6" };

        constexpr std::array<std::pair<Control, std::string_view>, 6> operateControls{ {
            { Control::checkContracts, "Check Contracts" },
            { Control::positionMode, "Position Mode" },
            { Control::start, "Start" },
            { Control::stop, "Stop" },
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

    void SimulatorPresenter::OnOperateAction(ui::model::ActionId action)
    {
        if (action == OperateAction::checkContracts)
            Post(arm_host::CheckContracts{});
        else if (action == OperateAction::positionMode)
            Post(arm_host::SelectMode{ can::FocMotorMode::position });
        else if (action == OperateAction::start)
            Post(arm_host::StartJoints{});
        else if (action == OperateAction::stop)
            Post(arm_host::StopJoints{});
        else if (action == OperateAction::clearFaults)
            Post(arm_host::ClearFaults{});
        else if (action == OperateAction::rearm && onRearmRequested)
            onRearmRequested(std::format("Re-arm after the trip by {}?", tripSource));
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

        const auto request = pending.find(event.command);
        if (request == pending.end() || (event.kind != arm_host::ArmEventKind::accepted && event.kind != arm_host::ArmEventKind::refused))
            return;

        if (event.kind == arm_host::ArmEventKind::accepted)
            shell.SetStatus(std::format("{}: accepted", request->second));
        else
            shell.SetStatus(std::format("{} refused: {}", request->second, event.reason));
        pending.erase(request);
    }

    void SimulatorPresenter::UpdateStatus()
    {
        status.target = TargetName(snapshot.target);
        status.realArm = snapshot.target.kind == arm_host::TargetKind::real;
        status.connection = snapshot.targetReady ? "Connected" : "Disconnected";
        status.connectionLevel = snapshot.targetReady ? ui::theme::StatusLevel::Ok : ui::theme::StatusLevel::Neutral;
        status.contracts = ContractsText(snapshot.contracts);
        const std::string_view source = tripSource.empty() ? std::string_view{ "unknown source" } : std::string_view{ tripSource };
        status.trip = snapshot.tripped ? std::format("Tripped — {}", source) : std::string{ "Trip: —" };
        status.tripLevel = snapshot.tripped ? ui::theme::StatusLevel::Fault : ui::theme::StatusLevel::Neutral;
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
