#ifndef TOOLS_SIMULATOR_VIEW_MODELS_CONFIGURATION_FLOW_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_CONFIGURATION_FLOW_HPP

#include "tools/arm_host/ArmCommands.hpp"
#include "tools/arm_host/ArmEvent.hpp"
#include "ui/model/FormModel.hpp"
#include "ui/shell/FormView.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <cstdint>
#include <span>
#include <string>

namespace simulator
{
    struct ConfigurationAction
    {
        static constexpr ui::model::ActionId read{ 0 };
        static constexpr ui::model::ActionId apply{ 1 };
        static constexpr ui::model::ActionId verify{ 2 };
        static constexpr ui::model::ActionId applyToAll{ 3 };
    };

    inline constexpr std::array<ui::model::ActionSpec, 4> configurationActions{ {
        { ConfigurationAction::read, "Read" },
        { ConfigurationAction::apply, "Apply", ui::theme::ButtonRole::Primary },
        { ConfigurationAction::verify, "Verify" },
        { ConfigurationAction::applyToAll, "Apply to All Joints" },
    } };

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

    class ConfigurationSubject
    {
    public:
        virtual arm_host::ArmCommand ReadCommand(std::size_t joint) const = 0;
        virtual arm_host::ArmCommand ApplyCommand(std::size_t joint) const = 0;
        virtual void Applied(std::size_t joint) = 0;
        virtual std::string Read(const arm_host::ArmEvent& event, bool verifying) = 0;

    protected:
        ~ConfigurationSubject() = default;
    };

    class ConfigurationFlow
    {
    public:
        struct EventKinds
        {
            arm_host::ArmEventKind applied;
            arm_host::ArmEventKind read;
        };

        ConfigurationFlow(arm_host::ArmCommands& arm, ui::shell::FormView& view, ConfigurationSubject& subject, EventKinds kinds);

        void Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events);
        void Trigger(ui::model::ActionId action);
        void JointChosen(std::size_t joint);
        void Edited();
        void SetNote(std::string note);
        void SetShown(bool shown);

        std::size_t Joint() const;
        bool HasApplied(std::size_t joint) const;
        ConfigurationState State() const;
        const std::string& StateText() const;
        ui::theme::StatusLevel StateLevel() const;
        const std::string& Hint() const;

    private:
        enum class Pending : uint8_t
        {
            none,
            read,
            apply,
            verify,
            applyToAll
        };

        void Handle(const arm_host::ArmEvent& event);
        void OnApplied(std::size_t joint);
        void OnRead(const arm_host::ArmEvent& event);
        void ReadAgain();
        void Failed(std::string_view reason);
        void Post(Pending operation, const arm_host::ArmCommand& command);
        void Enter(ConfigurationState state);
        void UpdateActions();

        arm_host::ArmCommands& arm;
        ui::shell::FormView& view;
        ConfigurationSubject& subject;
        EventKinds kinds;
        arm_host::ArmSnapshot snapshot;
        std::array<bool, arm::jointCount> applied{};
        Pending pending{ Pending::none };
        uint32_t postedCommand{ 0 };
        std::size_t joint{ 0 };
        ConfigurationState state{ ConfigurationState::unknown };
        std::string stateText;
        std::string note;
        std::string detail;
        std::string hint;
        bool shown{ true };
    };
}

#endif
