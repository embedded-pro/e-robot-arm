#include "tools/simulator/view_models/MotorControllerPresenter.hpp"
#include "e-foc-protocol/contract/FocMotorWireContract.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include <format>

namespace simulator
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::FieldKind;
        using ui::model::GroupId;
        using ui::model::OptionSpec;

        constexpr GroupId jointGroup{ 0 };
        constexpr GroupId currentGroup{ 1 };
        constexpr GroupId speedGroup{ 2 };
        constexpr GroupId positionGroup{ 3 };

        constexpr std::array<ui::model::GroupSpec, 4> groups{ {
            { jointGroup, "Joint" },
            { currentGroup, "Current loop" },
            { speedGroup, "Speed loop" },
            { positionGroup, "Position loop" },
        } };

        constexpr std::array<OptionSpec, arm::jointCount> jointOptions{ { { "J1" }, { "J2" }, { "J3" }, { "J4" }, { "J5" }, { "J6" } } };
        constexpr std::array<OptionSpec, 4> currentOptions{ { { "PID" }, { "Decoupled PID" }, { "Deadbeat" }, { "Sliding mode" } } };
        constexpr std::array<OptionSpec, 4> speedOptions{ { { "PID" }, { "LQI" }, { "ADRC" }, { "Two-DOF" } } };
        constexpr std::array<OptionSpec, 5> positionOptions{ { { "PID" }, { "Cascade P" }, { "LQR" }, { "LQI" }, { "Two-DOF" } } };

        constexpr std::array<ActionSpec, 4> actions{ {
            { MotorControllerAction::read, "Read" },
            { MotorControllerAction::apply, "Apply", ui::theme::ButtonRole::Primary },
            { MotorControllerAction::verify, "Verify" },
            { MotorControllerAction::applyToAll, "Apply to All Joints" },
        } };

        struct GainView
        {
            std::string_view label;
            std::string_view suffix;
            uint8_t decimals;
        };

        constexpr std::array<GainView, arm::gainCount> gainViews{ {
            { "Bandwidth", " rad/s", 2 },
            { "Switching gain", " A", 3 },
            { "Boundary layer", " A", 3 },
            { "Two-step deadbeat", "", 0 },
            { "Bandwidth", " rad/s", 2 },
            { "Error weight", "", 3 },
            { "Integral weight", "", 3 },
            { "Observer bandwidth ratio", "", 2 },
            { "Reference time constant", " s", 4 },
            { "Bandwidth", " rad/s", 2 },
            { "Position error weight", "", 3 },
            { "Speed error weight", "", 3 },
            { "Integral weight", "", 3 },
            { "Reference time constant", " s", 4 },
        } };

        constexpr std::array<std::string_view, arm::gainCount> gainNames{ "Current bandwidth", "Current switching gain", "Current boundary layer",
            "Current two-step deadbeat", "Speed bandwidth", "Speed error weight", "Speed integral weight", "Speed observer bandwidth ratio",
            "Speed reference time constant", "Position bandwidth", "Position error weight", "Position speed error weight", "Position integral weight",
            "Position reference time constant" };

        constexpr std::array<std::string_view, 3> modeNames{ "torque", "speed", "position" };

        ui::model::FieldId LoopField(arm::Gain gain)
        {
            const auto index = static_cast<std::size_t>(gain);
            if (index < static_cast<std::size_t>(arm::Gain::speedBandwidth))
                return MotorControllerField::current;
            if (index < static_cast<std::size_t>(arm::Gain::positionBandwidth))
                return MotorControllerField::speed;
            return MotorControllerField::position;
        }

        std::size_t OptionCount(ui::model::FieldId loop)
        {
            if (loop == MotorControllerField::current)
                return currentOptions.size();
            if (loop == MotorControllerField::speed)
                return speedOptions.size();
            return positionOptions.size();
        }

        arm::ControllerSettings WithAlgorithm(ui::model::FieldId loop, std::size_t algorithm)
        {
            arm::ControllerSettings settings{};
            if (loop == MotorControllerField::current)
                settings.current = static_cast<can::FocCurrentAlgorithm>(algorithm);
            else if (loop == MotorControllerField::speed)
                settings.speed = static_cast<can::FocSpeedAlgorithm>(algorithm);
            else
                settings.position = static_cast<can::FocPositionAlgorithm>(algorithm);
            return settings;
        }

        ui::model::Condition VisibleWhen(arm::Gain gain)
        {
            const auto loop = LoopField(gain);
            uint32_t mask{ 0 };
            for (std::size_t algorithm = 0; algorithm != OptionCount(loop); ++algorithm)
                if (arm::Uses(WithAlgorithm(loop, algorithm), gain))
                    mask |= 1u << algorithm;

            if (mask == (1u << OptionCount(loop)) - 1)
                return {};
            return { loop, mask };
        }

        ui::model::FieldSpec GainField(std::size_t index)
        {
            const auto gain = static_cast<arm::Gain>(index);
            const auto& view = gainViews[index];
            const auto descriptor = can::FindParameter(arm::gainParameters[index]);
            const arm::ControllerSettings defaults{};
            const auto group = LoopField(gain) == MotorControllerField::current ? currentGroup
                               : LoopField(gain) == MotorControllerField::speed ? speedGroup
                                                                                : positionGroup;
            return { MotorControllerField::Gain(gain), group, gain == arm::Gain::currentTwoStepDeadbeat ? FieldKind::Toggle : FieldKind::Number, view.label,
                view.suffix, { descriptor->minimum, descriptor->maximum, view.decimals == 2 ? 1.0 : 0.01, defaults.gains[index], view.decimals }, {},
                VisibleWhen(gain), {} };
        }

        std::string Format(float value)
        {
            return std::format("{:g}", value);
        }
    }

    MotorControllerPresenter::Form::Form()
        : spec{ groups, fields, actions, {} }
        , model(spec, values, {})
    {
        fields[0] = { MotorControllerField::joint, jointGroup, FieldKind::Choice, "Joint", "", {}, jointOptions, {}, {} };
        fields[1] = { MotorControllerField::current, currentGroup, FieldKind::Choice, "Algorithm", "", {}, currentOptions, {}, {} };
        fields[2] = { MotorControllerField::speed, speedGroup, FieldKind::Choice, "Algorithm", "", {}, speedOptions, {}, {} };
        fields[3] = { MotorControllerField::position, positionGroup, FieldKind::Choice, "Algorithm", "", {}, positionOptions, {}, {} };
        for (std::size_t gain = 0; gain != arm::gainCount; ++gain)
            fields[4 + gain] = GainField(gain);
        model.ResetToDefaults();
    }

    MotorControllerPresenter::MotorControllerPresenter(arm_host::ArmCommands& arm, ui::shell::FormView& view)
        : arm(arm)
        , view(view)
    {
        form.model.onActionTriggered = [this](ui::model::ActionId action)
        {
            OnAction(action);
        };
        view.Build(form.model);
        viewFieldChanged = form.model.onFieldChanged;
        form.model.onFieldChanged = [this](ui::model::FieldId field)
        {
            if (viewFieldChanged)
                viewFieldChanged(field);
            OnFieldChanged(field);
        };
        Load(arm::ControllerSettings{});
        Enter(ConfigurationState::unknown);
    }

    void MotorControllerPresenter::Refresh(const arm_host::ArmSnapshot& newSnapshot, std::span<const arm_host::ArmEvent> events)
    {
        snapshot = newSnapshot;
        for (const auto& event : events)
            Handle(event);
        UpdateActions();
    }

    void MotorControllerPresenter::SelectJoint(std::size_t newJoint)
    {
        if (newJoint >= arm::jointCount || newJoint == joint)
            return;

        form.model.SetSelection(MotorControllerField::joint, newJoint);
        view.Refresh(MotorControllerField::joint);
    }

    std::size_t MotorControllerPresenter::Joint() const
    {
        return joint;
    }

    ConfigurationState MotorControllerPresenter::State() const
    {
        return state;
    }

    const std::string& MotorControllerPresenter::StateText() const
    {
        return stateText;
    }

    ui::theme::StatusLevel MotorControllerPresenter::StateLevel() const
    {
        using enum ConfigurationState;
        if (state == verified)
            return ui::theme::StatusLevel::Ok;
        if (state == differs || state == failed)
            return ui::theme::StatusLevel::Fault;
        if (state == edited)
            return ui::theme::StatusLevel::Warning;
        return ui::theme::StatusLevel::Neutral;
    }

    const std::string& MotorControllerPresenter::Hint() const
    {
        return hint;
    }

    arm::ControllerSettings MotorControllerPresenter::FormSettings() const
    {
        arm::ControllerSettings settings{};
        settings.current = static_cast<can::FocCurrentAlgorithm>(form.model.Selection(MotorControllerField::current));
        settings.speed = static_cast<can::FocSpeedAlgorithm>(form.model.Selection(MotorControllerField::speed));
        settings.position = static_cast<can::FocPositionAlgorithm>(form.model.Selection(MotorControllerField::position));
        for (std::size_t gain = 0; gain != arm::gainCount; ++gain)
        {
            const auto field = MotorControllerField::Gain(static_cast<arm::Gain>(gain));
            settings.gains[gain] = static_cast<arm::Gain>(gain) == arm::Gain::currentTwoStepDeadbeat ? (form.model.Flag(field) ? 1.0f : 0.0f) : form.model.Float(field);
        }
        return settings;
    }

    void MotorControllerPresenter::OnAction(ui::model::ActionId action)
    {
        if (action == MotorControllerAction::read)
            Post(Pending::read, arm_host::ReadController{ joint });
        else if (action == MotorControllerAction::apply)
            Post(Pending::apply, arm_host::ApplyController{ joint, FormSettings() });
        else if (action == MotorControllerAction::verify && applied[joint].has_value())
            Post(Pending::verify, arm_host::ReadController{ joint });
        else if (action == MotorControllerAction::applyToAll)
            Post(Pending::applyToAll, arm_host::ApplyController{ 0, FormSettings() });
    }

    void MotorControllerPresenter::OnFieldChanged(ui::model::FieldId field)
    {
        if (loading)
            return;

        if (field == MotorControllerField::joint)
        {
            joint = form.model.Selection(MotorControllerField::joint);
            if (onJointSelected)
                onJointSelected(joint);
            if (pending == Pending::none && snapshot.state == arm::ArmState::ready)
                Post(Pending::read, arm_host::ReadController{ joint });
            else
                Enter(ConfigurationState::unknown);
            return;
        }

        if (state != ConfigurationState::unknown)
            Enter(ConfigurationState::edited);
        UpdateActions();
    }

    void MotorControllerPresenter::Handle(const arm_host::ArmEvent& event)
    {
        using enum arm_host::ArmEventKind;
        if (pending == Pending::none)
            return;

        if (event.kind == refused && event.command == postedCommand)
            Failed(event.reason);
        else if (event.kind == controllerApplied)
            Applied(event.joint);
        else if (event.kind == controllerRead && event.joint == joint)
            Read(event.joint, event.controller);
        else if (event.kind == controllerFailed)
            Failed(event.reason);
    }

    void MotorControllerPresenter::Applied(std::size_t appliedJoint)
    {
        applied[appliedJoint] = FormSettings();
        if (pending == Pending::applyToAll && appliedJoint + 1 < arm::jointCount)
        {
            postedCommand = arm.Post(arm_host::ApplyController{ appliedJoint + 1, FormSettings() });
            stateText = std::format("Applying… (J{} of {})", appliedJoint + 2, arm::jointCount);
            return;
        }

        pending = Pending::none;
        Enter(ConfigurationState::applied);
    }

    void MotorControllerPresenter::Read(std::size_t readJoint, const arm::ControllerReading& reading)
    {
        reportedMode = std::string{ modeNames[static_cast<std::size_t>(reading.mode)] };
        const auto operation = pending;
        pending = Pending::none;

        if (operation == Pending::verify && applied[readJoint].has_value())
        {
            detail = Mismatches(*applied[readJoint], reading.settings);
            Enter(detail.empty() ? ConfigurationState::verified : ConfigurationState::differs);
            return;
        }

        Load(reading.settings);
        Enter(ConfigurationState::read);
    }

    void MotorControllerPresenter::Failed(std::string_view reason)
    {
        pending = Pending::none;
        detail = std::string{ reason };
        Enter(ConfigurationState::failed);
    }

    void MotorControllerPresenter::Post(Pending operation, const arm_host::ArmCommand& command)
    {
        pending = operation;
        detail.clear();
        postedCommand = arm.Post(command);
        Enter(operation == Pending::read     ? ConfigurationState::reading
              : operation == Pending::verify ? ConfigurationState::verifying
                                             : ConfigurationState::applying);
        UpdateActions();
    }

    void MotorControllerPresenter::Load(const arm::ControllerSettings& settings)
    {
        loading = true;
        form.model.SetSelection(MotorControllerField::current, static_cast<std::size_t>(settings.current));
        form.model.SetSelection(MotorControllerField::speed, static_cast<std::size_t>(settings.speed));
        form.model.SetSelection(MotorControllerField::position, static_cast<std::size_t>(settings.position));
        for (std::size_t gain = 0; gain != arm::gainCount; ++gain)
        {
            const auto field = MotorControllerField::Gain(static_cast<arm::Gain>(gain));
            if (static_cast<arm::Gain>(gain) == arm::Gain::currentTwoStepDeadbeat)
                form.model.SetFlag(field, settings.gains[gain] != 0.0f);
            else
                form.model.SetNumber(field, settings.gains[gain]);
        }
        loading = false;
        view.Refresh();
    }

    void MotorControllerPresenter::Enter(ConfigurationState newState)
    {
        static constexpr std::array<std::string_view, 10> names{ "Not read", "Reading…", "Read", "Edited", "Applying…", "Applied", "Verifying…", "Verified",
            "Differs", "Failed" };

        state = newState;
        stateText = std::string{ names[static_cast<std::size_t>(state)] };
        if (!reportedMode.empty() && state != ConfigurationState::unknown)
            stateText += std::format(" — mode {} (reported)", reportedMode);
    }

    void MotorControllerPresenter::UpdateActions()
    {
        const bool ready = snapshot.targetReady && snapshot.state == arm::ArmState::ready;
        const bool idle = pending == Pending::none;
        const std::string blocked = !ready ? std::format("Not ready: {}", arm_host::StateName(snapshot.state)) : !idle ? std::string{ "Busy" }
                                                                                                                       : std::string{};

        for (const auto& action : actions)
            view.SetActionEnabled(action.id, blocked.empty() && (action.id != MotorControllerAction::verify || applied[joint].has_value()));

        hint.clear();
        if (!blocked.empty())
            hint = std::format("Read, Apply, Verify: {}", blocked);
        else if (!applied[joint].has_value())
            hint = "Verify: Nothing applied yet";
        if (!detail.empty())
            hint += std::format("{}{}", hint.empty() ? "" : "\n", detail);
    }

    std::string MotorControllerPresenter::Mismatches(const arm::ControllerSettings& expected, const arm::ControllerSettings& actual) const
    {
        const auto mismatches = arm::Compare(expected, actual);
        std::string text;
        const auto add = [&text](std::string_view line)
        {
            text += std::format("{}{}", text.empty() ? "" : "\n", line);
        };

        if (mismatches.current)
            add(std::format("Current algorithm: applied {}, joint reports {}", currentOptions[static_cast<std::size_t>(expected.current)].label,
                currentOptions[static_cast<std::size_t>(actual.current)].label));
        if (mismatches.speed)
            add(std::format("Speed algorithm: applied {}, joint reports {}", speedOptions[static_cast<std::size_t>(expected.speed)].label,
                speedOptions[static_cast<std::size_t>(actual.speed)].label));
        if (mismatches.position)
            add(std::format("Position algorithm: applied {}, joint reports {}", positionOptions[static_cast<std::size_t>(expected.position)].label,
                positionOptions[static_cast<std::size_t>(actual.position)].label));
        for (std::size_t gain = 0; gain != arm::gainCount; ++gain)
            if (mismatches.gains[gain])
                add(std::format("{}: applied {}, joint reports {}", gainNames[gain], Format(expected.gains[gain]), Format(actual.gains[gain])));
        return text;
    }
}
