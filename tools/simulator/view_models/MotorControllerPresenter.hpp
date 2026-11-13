#ifndef TOOLS_SIMULATOR_VIEW_MODELS_MOTOR_CONTROLLER_PRESENTER_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_MOTOR_CONTROLLER_PRESENTER_HPP

#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "tools/arm_host/ArmCommands.hpp"
#include "tools/simulator/view_models/ConfigurationFlow.hpp"
#include "ui/core/Callback.hpp"
#include "ui/model/FormModel.hpp"
#include "ui/shell/FormView.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <optional>
#include <span>
#include <string>

namespace simulator
{
    struct MotorControllerField
    {
        static constexpr ui::model::FieldId joint{ 0 };
        static constexpr ui::model::FieldId current{ 1 };
        static constexpr ui::model::FieldId speed{ 2 };
        static constexpr ui::model::FieldId position{ 3 };
        static constexpr uint16_t firstGain{ 10 };

        static constexpr ui::model::FieldId Gain(arm::Gain gain)
        {
            return ui::model::FieldId{ static_cast<uint16_t>(firstGain + static_cast<uint16_t>(gain)) };
        }
    };

    using MotorControllerAction = ConfigurationAction;

    class MotorControllerPresenter
        : private ConfigurationSubject
    {
    public:
        MotorControllerPresenter(arm_host::ArmCommands& arm, ui::shell::FormView& view);
        MotorControllerPresenter(const MotorControllerPresenter& other) = delete;
        MotorControllerPresenter& operator=(const MotorControllerPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events);
        void SelectJoint(std::size_t joint);
        void SetJointSelected(const ui::Callback<void(std::size_t joint)>& handler);
        void SetShown(bool shown);

        std::size_t Joint() const;
        ConfigurationState State() const;
        const std::string& StateText() const;
        ui::theme::StatusLevel StateLevel() const;
        const std::string& Hint() const;
        arm::ControllerSettings FormSettings() const;

    private:
        struct Form
        {
            Form();
            Form(const Form& other) = delete;
            Form& operator=(const Form& other) = delete;

            std::array<ui::model::FieldSpec, 4 + arm::gainCount> fields;
            ui::model::FormSpec spec;
            std::array<ui::model::FieldValue, 4 + arm::gainCount> values{};
            ui::model::FormModel model;
        };

        arm_host::ArmCommand ReadCommand(std::size_t joint) const override;
        arm_host::ArmCommand ApplyCommand(std::size_t joint) const override;
        void Applied(std::size_t joint) override;
        std::string Read(const arm_host::ArmEvent& event, bool verifying) override;

        void OnFieldChanged(ui::model::FieldId field);
        void Load(const arm::ControllerSettings& settings);
        std::string Mismatches(const arm::ControllerSettings& expected, const arm::ControllerSettings& actual) const;

        ui::shell::FormView& view;
        Form form;
        ConfigurationFlow flow;
        ui::Callback<void(ui::model::FieldId)> viewFieldChanged;
        ui::Callback<void(std::size_t joint)> jointSelected;
        std::array<arm::ControllerSettings, arm::jointCount> applied{};
        bool loading{ false };
    };
}

#endif
