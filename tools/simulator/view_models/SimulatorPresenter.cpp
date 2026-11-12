#include "tools/simulator/view_models/SimulatorPresenter.hpp"
#include <cctype>

namespace simulator
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::theme::ButtonRole;

        constexpr std::array<ActionSpec, 6> operateActions{ {
            { SimulatorPresenter::checkContracts, "Check Contracts" },
            { SimulatorPresenter::positionMode, "Position Mode" },
            { SimulatorPresenter::start, "Start", ButtonRole::Start },
            { SimulatorPresenter::stop, "Stop", ButtonRole::Stop },
            { SimulatorPresenter::clearFaults, "Clear Faults", ButtonRole::Reset },
            { SimulatorPresenter::rearm, "Re-arm…" },
        } };

        constexpr std::array<ActionSpec, 2> moveActions{ {
            { SimulatorPresenter::move, "Move", ButtonRole::Primary },
            { SimulatorPresenter::abort, "Abort", ButtonRole::Stop },
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
        , moveFields(MoveFields(limits))
        , operateSpec{ {}, {}, operateActions, {} }
        , moveSpec{ {}, moveFields, moveActions, {} }
        , operateModel(operateSpec, {}, {})
        , moveModel(moveSpec, moveValues, {})
        , log(start)
    {
        operateModel.onActionTriggered = [this](ui::model::ActionId action)
        {
            OnOperateAction(action);
        };
        moveModel.onActionTriggered = [this](ui::model::ActionId action)
        {
            OnMoveAction(action);
        };

        operateView.Build(operateModel);
        moveView.Build(moveModel);
        Refresh(snapshot, {});
    }

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
        return moveModel;
    }

    void SimulatorPresenter::OnOperateAction(ui::model::ActionId action)
    {
        if (action == checkContracts)
            Post(arm_host::CheckContracts{});
        else if (action == positionMode)
            Post(arm_host::SelectMode{ can::FocMotorMode::position });
        else if (action == start)
            Post(arm_host::StartJoints{});
        else if (action == stop)
            Post(arm_host::StopJoints{});
        else if (action == clearFaults)
            Post(arm_host::ClearFaults{});
        else if (action == rearm && onRearmRequested)
            onRearmRequested("Re-arm after the trip by " + tripSource + "?");
    }

    void SimulatorPresenter::OnMoveAction(ui::model::ActionId action)
    {
        if (action == abort)
            return Post(arm_host::AbortMove{});

        if (action != move || !State(Control::move).enabled)
            return;

        arm::JointPositions target{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            target[joint] = foc::Radians{ moveModel.Float(moveFields[joint].id) };
        Post(arm_host::MoveTo{ target });
    }

    void SimulatorPresenter::Post(const arm_host::ArmCommand& command)
    {
        const auto name = SentenceCase(arm_host::CommandName(command));
        pending[arm.Post(command)] = name;
        shell.SetStatus(name + " requested…");
    }

    void SimulatorPresenter::Handle(const arm_host::ArmEvent& event)
    {
        log.Add(event);

        if (event.kind == arm_host::ArmEventKind::tripped)
            tripSource = "Joint " + std::to_string(event.joint + 1) +
                         (event.fault == can::FocFaultCode::none ? std::string{ " (silent)" } : " (" + std::string{ FaultName(event.fault) } + ")");

        const auto request = pending.find(event.command);
        if (request == pending.end() || (event.kind != arm_host::ArmEventKind::accepted && event.kind != arm_host::ArmEventKind::refused))
            return;

        shell.SetStatus(event.kind == arm_host::ArmEventKind::accepted ? request->second + ": accepted"
                                                                       : request->second + " refused: " + std::string{ event.reason });
        pending.erase(request);
    }

    void SimulatorPresenter::UpdateStatus()
    {
        status.target = TargetName(snapshot.target);
        status.realArm = snapshot.target.kind == arm_host::TargetKind::real;
        status.connection = snapshot.targetReady ? "Connected" : "Disconnected";
        status.connectionLevel = snapshot.targetReady ? ui::theme::StatusLevel::Ok : ui::theme::StatusLevel::Neutral;
        status.contracts = ContractsText(snapshot.contracts);
        status.trip = snapshot.tripped ? "Tripped — " + (tripSource.empty() ? std::string{ "unknown source" } : tripSource) : "Trip: —";
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
        const auto violation = moveModel.Validate();
        if (!violation)
            return {};
        return std::string{ jointLabels[violation->field.value] } + " outside its limits";
    }

    std::string SimulatorPresenter::Hint(std::span<const std::pair<Control, std::string_view>> controls) const
    {
        std::string hint;
        for (const auto& [control, label] : controls)
            if (!State(control).enabled)
                hint += (hint.empty() ? "" : "\n") + std::string{ label } + ": " + State(control).reason;
        return hint;
    }
}
