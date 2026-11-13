#include "tools/simulator/view_models/MotorPresenter.hpp"
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
        constexpr GroupId modelGroup{ 1 };
        constexpr GroupId limitsGroup{ 2 };

        constexpr std::array<ui::model::GroupSpec, 3> groups{ {
            { jointGroup, "Joint" },
            { modelGroup, "Motor model" },
            { limitsGroup, "Drive limits" },
        } };

        constexpr std::array<OptionSpec, arm::jointCount> jointOptions{ { { "J1" }, { "J2" }, { "J3" }, { "J4" }, { "J5" }, { "J6" } } };
        constexpr std::array<OptionSpec, arm::motorPresets.size() + 1> presetOptions{ { { arm::motorPresets[0].name }, { arm::motorPresets[1].name }, { "Custom" } } };
        constexpr std::array<std::string_view, arm::motorParameterCount> suffixes{ " Ω", " mH", "", " mWb", " g·cm²", " µN·m·s/rad", " mN·m", " A", " A", " s",
            " rad/s" };

        ui::model::FieldSpec ParameterField(std::size_t index)
        {
            const auto parameter = static_cast<arm::MotorParameter>(index);
            const auto& unit = arm::motorUnits[index];
            const auto descriptor = can::FindParameter(arm::motorParameters[index]);
            const auto scale = static_cast<double>(unit.scale);
            const bool whole = parameter == arm::MotorParameter::polePairs;
            return { MotorField::Parameter(parameter), index < arm::motorModelCount ? modelGroup : limitsGroup, FieldKind::Number, unit.label, suffixes[index],
                { descriptor->minimum * scale, descriptor->maximum * scale, whole ? 1.0 : 0.01, arm::motorPresets[0].settings.values[index] * scale,
                    static_cast<uint8_t>(whole ? 0 : 3) },
                {}, {}, {} };
        }

        std::optional<std::size_t> MatchingPreset(const arm::MotorSettings& motor)
        {
            for (std::size_t preset = 0; preset != arm::motorPresets.size(); ++preset)
                if (!arm::Compare(arm::motorPresets[preset].settings, motor).Any())
                    return preset;
            return std::nullopt;
        }

        std::string Format(float value, std::size_t parameter)
        {
            return std::format("{:g}{}", value * arm::motorUnits[parameter].scale, suffixes[parameter]);
        }
    }

    MotorPresenter::Form::Form()
        : spec{ groups, fields, configurationActions, {} }
        , model(spec, values, {})
    {
        fields[0] = { MotorField::joint, jointGroup, FieldKind::Choice, "Joint", "", {}, jointOptions, {}, {} };
        fields[1] = { MotorField::preset, jointGroup, FieldKind::Choice, "Preset", "", {}, presetOptions, {}, {} };
        for (std::size_t parameter = 0; parameter != arm::motorParameterCount; ++parameter)
            fields[2 + parameter] = ParameterField(parameter);
        model.ResetToDefaults();
    }

    MotorPresenter::MotorPresenter(arm_host::ArmCommands& arm, ui::shell::FormView& view)
        : view(view)
        , flow(arm, view, *this, { arm_host::ArmEventKind::motorApplied, arm_host::ArmEventKind::motorRead })
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
        Load(arm::motorPresets[0].settings, 0);
    }

    void MotorPresenter::Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events)
    {
        flow.Refresh(snapshot, events);
    }

    void MotorPresenter::SelectJoint(std::size_t newJoint)
    {
        if (newJoint >= arm::jointCount || newJoint == flow.Joint())
            return;

        form.model.SetSelection(MotorField::joint, newJoint);
        view.Refresh(MotorField::joint);
    }

    void MotorPresenter::SetJointSelected(const ui::Callback<void(std::size_t joint)>& handler)
    {
        jointSelected = handler;
    }

    void MotorPresenter::SetShown(bool shown)
    {
        flow.SetShown(shown);
    }

    std::size_t MotorPresenter::Joint() const
    {
        return flow.Joint();
    }

    ConfigurationState MotorPresenter::State() const
    {
        return flow.State();
    }

    const std::string& MotorPresenter::StateText() const
    {
        return flow.StateText();
    }

    ui::theme::StatusLevel MotorPresenter::StateLevel() const
    {
        return flow.StateLevel();
    }

    const std::string& MotorPresenter::Hint() const
    {
        return flow.Hint();
    }

    std::size_t MotorPresenter::Preset() const
    {
        return form.model.Selection(MotorField::preset);
    }

    arm::MotorSettings MotorPresenter::FormSettings() const
    {
        arm::MotorSettings motor{};
        for (std::size_t parameter = 0; parameter != arm::motorParameterCount; ++parameter)
            motor.values[parameter] = static_cast<float>(form.model.Number(MotorField::Parameter(static_cast<arm::MotorParameter>(parameter))) /
                                                         static_cast<double>(arm::motorUnits[parameter].scale));
        return motor;
    }

    arm_host::ArmCommand MotorPresenter::ReadCommand(std::size_t joint) const
    {
        return arm_host::ReadMotor{ joint };
    }

    arm_host::ArmCommand MotorPresenter::ApplyCommand(std::size_t joint) const
    {
        return arm_host::ApplyMotor{ joint, FormSettings() };
    }

    void MotorPresenter::Applied(std::size_t joint)
    {
        applied[joint] = FormSettings();
    }

    std::string MotorPresenter::Read(const arm_host::ArmEvent& event, bool verifying)
    {
        if (verifying)
            return Differences(applied[event.joint], event.motor, true);

        const auto preset = MatchingPreset(event.motor);
        basePreset = preset;
        Load(event.motor, preset.value_or(customPreset));
        return {};
    }

    void MotorPresenter::OnFieldChanged(ui::model::FieldId field)
    {
        if (loading)
            return;

        if (field == MotorField::joint)
        {
            const auto joint = form.model.Selection(MotorField::joint);
            if (jointSelected)
                jointSelected(joint);
            flow.JointChosen(joint);
            return;
        }

        if (field == MotorField::preset && Preset() != customPreset)
        {
            basePreset = Preset();
            Load(arm::motorPresets[Preset()].settings, Preset());
        }
        else if (field != MotorField::preset)
        {
            const auto matching = MatchingPreset(FormSettings());
            loading = true;
            form.model.SetSelection(MotorField::preset, matching.value_or(customPreset));
            loading = false;
            view.Refresh(MotorField::preset);
            UpdateNote();
        }

        flow.Edited();
    }

    void MotorPresenter::Load(const arm::MotorSettings& motor, std::size_t preset)
    {
        loading = true;
        form.model.SetSelection(MotorField::preset, preset);
        for (std::size_t parameter = 0; parameter != arm::motorParameterCount; ++parameter)
            form.model.SetNumber(MotorField::Parameter(static_cast<arm::MotorParameter>(parameter)),
                static_cast<double>(motor.values[parameter]) * static_cast<double>(arm::motorUnits[parameter].scale));
        loading = false;
        view.Refresh();
        UpdateNote();
    }

    void MotorPresenter::UpdateNote()
    {
        if (Preset() != customPreset)
            flow.SetNote(std::string{ arm::motorPresets[Preset()].name });
        else if (basePreset.has_value())
            flow.SetNote(std::format("custom: differs from {} in {}", arm::motorPresets[*basePreset].name,
                Differences(arm::motorPresets[*basePreset].settings, FormSettings(), false)));
        else
            flow.SetNote("custom");
    }

    std::string MotorPresenter::Differences(const arm::MotorSettings& expected, const arm::MotorSettings& actual, bool withValues) const
    {
        const auto mismatches = arm::Compare(expected, actual);
        std::string text;
        for (std::size_t parameter = 0; parameter != arm::motorParameterCount; ++parameter)
        {
            if (!mismatches.values[parameter])
                continue;
            const auto label = arm::motorUnits[parameter].label;
            if (withValues)
                text += std::format("{}{}: applied {}, joint reports {}", text.empty() ? "" : "\n", label, Format(expected.values[parameter], parameter),
                    Format(actual.values[parameter], parameter));
            else
                text += std::format("{}{}", text.empty() ? "" : ", ", label);
        }
        return text;
    }
}
