#ifndef TOOLS_SIMULATOR_VIEW_MODELS_MOTOR_CONTROLLER_PRESENTER_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_MOTOR_CONTROLLER_PRESENTER_HPP

#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "tools/arm_host/ArmCommands.hpp"
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

    struct MotorControllerAction
    {
        static constexpr ui::model::ActionId read{ 0 };
        static constexpr ui::model::ActionId apply{ 1 };
        static constexpr ui::model::ActionId verify{ 2 };
        static constexpr ui::model::ActionId applyToAll{ 3 };
    };

    enum class ConfigurationState : uint8_t
    {
        unknown,
        reading,
        read,
        edited,
        applying,
        applied,
        verifying,
        verified,
        differs,
        failed
    };

    class MotorControllerPresenter
    {
    public:
        MotorControllerPresenter(arm_host::ArmCommands& arm, ui::shell::FormView& view);
        MotorControllerPresenter(const MotorControllerPresenter& other) = delete;
        MotorControllerPresenter& operator=(const MotorControllerPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events);
        void SelectJoint(std::size_t joint);

        std::size_t Joint() const;
        ConfigurationState State() const;
        const std::string& StateText() const;
        ui::theme::StatusLevel StateLevel() const;
        const std::string& Hint() const;
        arm::ControllerSettings FormSettings() const;

        void SetJointSelected(const ui::Callback<void(std::size_t joint)>& handler);

    private:
        enum class Pending : uint8_t
        {
            none,
            read,
            apply,
            verify,
            applyToAll
        };

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

        void OnAction(ui::model::ActionId action);
        void OnFieldChanged(ui::model::FieldId field);
        void Handle(const arm_host::ArmEvent& event);
        void Applied(std::size_t joint);
        void Read(std::size_t joint, const arm::ControllerReading& reading);
        void Failed(std::string_view reason);
        void Post(Pending operation, const arm_host::ArmCommand& command);
        void Load(const arm::ControllerSettings& settings);
        void Enter(ConfigurationState state);
        void UpdateActions();
        std::string Mismatches(const arm::ControllerSettings& expected, const arm::ControllerSettings& actual) const;

        arm_host::ArmCommands& arm;
        ui::shell::FormView& view;
        Form form;
        ui::Callback<void(ui::model::FieldId)> viewFieldChanged;
        ui::Callback<void(std::size_t joint)> jointSelected;
        arm_host::ArmSnapshot snapshot;
        std::array<std::optional<arm::ControllerSettings>, arm::jointCount> applied{};
        Pending pending{ Pending::none };
        uint32_t postedCommand{ 0 };
        std::size_t joint{ 0 };
        bool loading{ false };
        ConfigurationState state{ ConfigurationState::unknown };
        std::string stateText;
        std::string detail;
        std::string hint;
        std::string reportedMode;
    };
}

#endif
