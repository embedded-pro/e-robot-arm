#include "tools/simulator/view_models/ConfigurationFlow.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include <format>

namespace simulator
{
    ConfigurationFlow::ConfigurationFlow(arm_host::ArmCommands& arm, ui::shell::FormView& view, ConfigurationSubject& subject, EventKinds kinds)
        : arm(arm)
        , view(view)
        , subject(subject)
        , kinds(kinds)
    {
        Enter(ConfigurationState::unknown);
    }

    void ConfigurationFlow::Refresh(const arm_host::ArmSnapshot& newSnapshot, std::span<const arm_host::ArmEvent> events)
    {
        snapshot = newSnapshot;
        for (const auto& event : events)
            Handle(event);
        UpdateActions();
    }

    void ConfigurationFlow::Trigger(ui::model::ActionId action)
    {
        if (action == ConfigurationAction::read)
            Post(Pending::read, subject.ReadCommand(joint));
        else if (action == ConfigurationAction::apply)
            Post(Pending::apply, subject.ApplyCommand(joint));
        else if (action == ConfigurationAction::verify && applied[joint])
            Post(Pending::verify, subject.ReadCommand(joint));
        else if (action == ConfigurationAction::applyToAll)
            Post(Pending::applyToAll, subject.ApplyCommand(0));
    }

    void ConfigurationFlow::JointChosen(std::size_t newJoint)
    {
        joint = newJoint;
        if (shown && pending == Pending::none && snapshot.state == arm::ArmState::ready)
            Post(Pending::read, subject.ReadCommand(joint));
        else
            Enter(ConfigurationState::unknown);
    }

    void ConfigurationFlow::SetShown(bool newShown)
    {
        shown = newShown;
        if (shown && state == ConfigurationState::unknown && pending == Pending::none && snapshot.targetReady && snapshot.state == arm::ArmState::ready)
            Post(Pending::read, subject.ReadCommand(joint));
    }

    void ConfigurationFlow::Edited()
    {
        if (state != ConfigurationState::unknown)
            Enter(ConfigurationState::edited);
        UpdateActions();
    }

    void ConfigurationFlow::SetNote(std::string newNote)
    {
        note = std::move(newNote);
        Enter(state);
    }

    std::size_t ConfigurationFlow::Joint() const
    {
        return joint;
    }

    bool ConfigurationFlow::HasApplied(std::size_t candidate) const
    {
        return applied[candidate];
    }

    ConfigurationState ConfigurationFlow::State() const
    {
        return state;
    }

    const std::string& ConfigurationFlow::StateText() const
    {
        return stateText;
    }

    ui::theme::StatusLevel ConfigurationFlow::StateLevel() const
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

    const std::string& ConfigurationFlow::Hint() const
    {
        return hint;
    }

    void ConfigurationFlow::Handle(const arm_host::ArmEvent& event)
    {
        if (pending == Pending::none)
            return;

        if (event.kind == arm_host::ArmEventKind::refused && event.command == postedCommand)
            Failed(event.reason);
        else if (event.kind == kinds.applied)
            OnApplied(event.joint);
        else if (event.kind == kinds.read && event.joint == joint)
            OnRead(event);
        else if (event.kind == kinds.read)
            ReadAgain();
        else if (event.kind == arm_host::ArmEventKind::controllerFailed)
            Failed(event.reason);
    }

    void ConfigurationFlow::OnApplied(std::size_t appliedJoint)
    {
        applied[appliedJoint] = true;
        subject.Applied(appliedJoint);
        if (pending == Pending::applyToAll && appliedJoint + 1 < arm::jointCount)
        {
            postedCommand = arm.Post(subject.ApplyCommand(appliedJoint + 1));
            stateText = std::format("Applying… (J{} of {})", appliedJoint + 2, arm::jointCount);
            return;
        }

        pending = Pending::none;
        Enter(ConfigurationState::applied);
    }

    void ConfigurationFlow::OnRead(const arm_host::ArmEvent& event)
    {
        const bool verifying = pending == Pending::verify && applied[event.joint];
        pending = Pending::none;
        detail = subject.Read(event, verifying);
        if (verifying)
            Enter(detail.empty() ? ConfigurationState::verified : ConfigurationState::differs);
        else
            Enter(ConfigurationState::read);
    }

    void ConfigurationFlow::ReadAgain()
    {
        pending = Pending::none;
        if (shown && snapshot.state == arm::ArmState::ready)
            Post(Pending::read, subject.ReadCommand(joint));
        else
            Enter(ConfigurationState::unknown);
    }

    void ConfigurationFlow::Failed(std::string_view reason)
    {
        pending = Pending::none;
        detail = std::string{ reason };
        Enter(ConfigurationState::failed);
    }

    void ConfigurationFlow::Post(Pending operation, const arm_host::ArmCommand& command)
    {
        pending = operation;
        detail.clear();
        postedCommand = arm.Post(command);
        if (operation == Pending::read)
            Enter(ConfigurationState::reading);
        else if (operation == Pending::verify)
            Enter(ConfigurationState::verifying);
        else
            Enter(ConfigurationState::applying);
        UpdateActions();
    }

    void ConfigurationFlow::Enter(ConfigurationState newState)
    {
        static constexpr std::array<std::string_view, 10> names{ "Not read", "Reading…", "Read", "Edited", "Applying…", "Applied", "Verifying…", "Verified",
            "Differs", "Failed" };

        state = newState;
        stateText = std::string{ names[static_cast<std::size_t>(state)] };
        if (!note.empty() && state != ConfigurationState::unknown)
            stateText += std::format(" — {}", note);
    }

    void ConfigurationFlow::UpdateActions()
    {
        const bool ready = snapshot.targetReady && snapshot.state == arm::ArmState::ready;
        std::string blocked;
        if (!ready)
            blocked = std::format("Not ready: {}", arm_host::StateName(snapshot.state));
        else if (pending != Pending::none)
            blocked = "Busy";

        for (const auto& action : configurationActions)
            view.SetActionEnabled(action.id, blocked.empty() && (action.id != ConfigurationAction::verify || applied[joint]));

        hint.clear();
        if (!blocked.empty())
            hint = std::format("Read, Apply, Verify: {}", blocked);
        else if (!applied[joint])
            hint = "Verify: Nothing applied yet";
        if (!detail.empty())
            hint += std::format("{}{}", hint.empty() ? "" : "\n", detail);
    }
}
