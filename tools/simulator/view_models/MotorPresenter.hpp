#ifndef TOOLS_SIMULATOR_VIEW_MODELS_MOTOR_PRESENTER_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_MOTOR_PRESENTER_HPP

#include "core/joint_configurator/interfaces/MotorSettings.hpp"
#include "tools/simulator/view_models/ConfigurationFlow.hpp"
#include "ui/core/Callback.hpp"
#include <array>
#include <optional>
#include <span>
#include <string>

namespace simulator
{
    struct MotorField
    {
        static constexpr ui::model::FieldId joint{ 0 };
        static constexpr ui::model::FieldId preset{ 1 };
        static constexpr uint16_t firstParameter{ 10 };

        static constexpr ui::model::FieldId Parameter(arm::MotorParameter parameter)
        {
            return ui::model::FieldId{ static_cast<uint16_t>(firstParameter + static_cast<uint16_t>(parameter)) };
        }
    };

    inline constexpr std::size_t customPreset{ arm::motorPresets.size() };

    class MotorPresenter
        : private ConfigurationSubject
    {
    public:
        MotorPresenter(arm_host::ArmCommands& arm, ui::shell::FormView& view);
        MotorPresenter(const MotorPresenter& other) = delete;
        MotorPresenter& operator=(const MotorPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events);
        void SelectJoint(std::size_t joint);
        void SetJointSelected(const ui::Callback<void(std::size_t joint)>& handler);
        void SetShown(bool shown);

        std::size_t Joint() const;
        ConfigurationState State() const;
        const std::string& StateText() const;
        ui::theme::StatusLevel StateLevel() const;
        const std::string& Hint() const;
        std::size_t Preset() const;
        arm::MotorSettings FormSettings() const;

    private:
        struct Form
        {
            Form();
            Form(const Form& other) = delete;
            Form& operator=(const Form& other) = delete;

            std::array<ui::model::FieldSpec, 2 + arm::motorParameterCount> fields;
            ui::model::FormSpec spec;
            std::array<ui::model::FieldValue, 2 + arm::motorParameterCount> values{};
            ui::model::FormModel model;
        };

        arm_host::ArmCommand ReadCommand(std::size_t joint) const override;
        arm_host::ArmCommand ApplyCommand(std::size_t joint) const override;
        void Applied(std::size_t joint) override;
        std::string Read(const arm_host::ArmEvent& event, bool verifying) override;

        void OnFieldChanged(ui::model::FieldId field);
        void Load(const arm::MotorSettings& motor, std::size_t preset);
        void UpdateNote();
        std::string Differences(const arm::MotorSettings& expected, const arm::MotorSettings& actual, bool withValues) const;

        ui::shell::FormView& view;
        Form form;
        ConfigurationFlow flow;
        ui::Callback<void(ui::model::FieldId)> viewFieldChanged;
        ui::Callback<void(std::size_t joint)> jointSelected;
        std::array<arm::MotorSettings, arm::jointCount> applied{};
        std::optional<std::size_t> basePreset{ 0 };
        bool loading{ false };
    };
}

#endif
