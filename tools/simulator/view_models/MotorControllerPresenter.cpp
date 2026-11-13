#include "tools/simulator/view_models/MotorControllerPresenter.hpp"
#include "e-foc-protocol/contract/FocMotorWireContract.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include <format>

namespace simulator
{
    namespace
    {
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

        ui::model::GroupId GroupOf(ui::model::FieldId loop)
        {
            if (loop == MotorControllerField::current)
                return currentGroup;
            if (loop == MotorControllerField::speed)
                return speedGroup;
            return positionGroup;
        }

        ui::model::FieldSpec GainField(std::size_t index)
        {
            const auto gain = static_cast<arm::Gain>(index);
            const auto& view = gainViews[index];
            const auto descriptor = can::FindParameter(arm::gainParameters[index]);
            const arm::ControllerSettings defaults{};
            const auto group = GroupOf(LoopField(gain));
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
        : spec{ groups, fields, configurationActions, {} }
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
        : view(view)
        , flow(arm, view, *this, { arm_host::ArmEventKind::controllerApplied, arm_host::ArmEventKind::controllerRead })
    {
        form.model.onActionTriggered = [this](ui::model::ActionId action)
        {
            flow.Trigger(action);
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
    }

    void MotorControllerPresenter::Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events)
    {
        flow.Refresh(snapshot, events);
    }

    void MotorControllerPresenter::SelectJoint(std::size_t newJoint)
    {
        if (newJoint >= arm::jointCount || newJoint == flow.Joint())
            return;

        form.model.SetSelection(MotorControllerField::joint, newJoint);
        view.Refresh(MotorControllerField::joint);
    }

    void MotorControllerPresenter::SetJointSelected(const ui::Callback<void(std::size_t joint)>& handler)
    {
        jointSelected = handler;
    }

    void MotorControllerPresenter::SetShown(bool shown)
    {
        flow.SetShown(shown);
    }

    std::size_t MotorControllerPresenter::Joint() const
    {
        return flow.Joint();
    }

    ConfigurationState MotorControllerPresenter::State() const
    {
        return flow.State();
    }

    const std::string& MotorControllerPresenter::StateText() const
    {
        return flow.StateText();
    }

    ui::theme::StatusLevel MotorControllerPresenter::StateLevel() const
    {
        return flow.StateLevel();
    }

    const std::string& MotorControllerPresenter::Hint() const
    {
        return flow.Hint();
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
            if (static_cast<arm::Gain>(gain) == arm::Gain::currentTwoStepDeadbeat)
                settings.gains[gain] = form.model.Flag(field) ? 1.0f : 0.0f;
            else
                settings.gains[gain] = form.model.Float(field);
        }
        return settings;
    }

    arm_host::ArmCommand MotorControllerPresenter::ReadCommand(std::size_t joint) const
    {
        return arm_host::ReadController{ joint };
    }

    arm_host::ArmCommand MotorControllerPresenter::ApplyCommand(std::size_t joint) const
    {
        return arm_host::ApplyController{ joint, FormSettings() };
    }

    void MotorControllerPresenter::Applied(std::size_t joint)
    {
        applied[joint] = FormSettings();
    }

    std::string MotorControllerPresenter::Read(const arm_host::ArmEvent& event, bool verifying)
    {
        flow.SetNote(std::format("mode {} (reported)", modeNames[static_cast<std::size_t>(event.controller.mode)]));
        if (verifying)
            return Mismatches(applied[event.joint], event.controller.settings);

        Load(event.controller.settings);
        return {};
    }

    void MotorControllerPresenter::OnFieldChanged(ui::model::FieldId field)
    {
        if (loading)
            return;

        if (field == MotorControllerField::joint)
        {
            const auto joint = form.model.Selection(MotorControllerField::joint);
            if (jointSelected)
                jointSelected(joint);
            flow.JointChosen(joint);
            return;
        }

        flow.Edited();
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
