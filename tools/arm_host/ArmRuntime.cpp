#include "tools/arm_host/ArmRuntime.hpp"

namespace arm_host
{
    ArmRuntime::ArmRuntime(const arm::ArmProfile& profile, const TargetOpener& opener, ArmSessionSink& sink, const ArmTarget& initial)
        : profile(profile)
        , opener(opener)
        , sink(sink)
        , poll(pollInterval, [this]()
              {
                  if (link)
                      link->Poll();
              })
    {
        Report(Open(initial) ? ArmEventKind::targetReady : ArmEventKind::targetFailed, 0);
    }

    void ArmRuntime::Execute(uint32_t id, const ArmCommand& command)
    {
        if (switching && session && std::holds_alternative<EmergencyStop>(command))
            session->Execute(id, command);
        else if (switching)
            sink.OnEvent({ .kind = ArmEventKind::refused, .time = infra::Now(), .command = id, .commandName = CommandName(command), .reason = "switching target" });
        else if (!session)
            sink.OnEvent({ .kind = ArmEventKind::refused, .time = infra::Now(), .command = id, .commandName = CommandName(command), .reason = "no target" });
        else
            session->Execute(id, command);
    }

    void ArmRuntime::SwitchTarget(uint32_t id, const ArmTarget& target)
    {
        if (switching)
            return Report(ArmEventKind::targetFailed, id, "switching target");
        if (session && session->Moving())
            return Report(ArmEventKind::targetFailed, id, "moving");

        switching = true;
        switchId = id;
        switchTarget = target;
        previousTarget = current;
        if (session)
            session->StopAll();

        grace.Start(stopGrace, [this]()
            {
                FinishSwitch();
            });
    }

    void ArmRuntime::LoadProfile(uint32_t id, const arm::ArmProfile& newProfile)
    {
        const auto refuse = [this, id](std::string_view reason)
        {
            sink.OnEvent({ .kind = ArmEventKind::refused, .time = infra::Now(), .command = id, .commandName = "load profile", .reason = reason });
        };

        if (switching)
            return refuse("switching target");
        if (session && session->Running())
            return refuse("the arm is enabled: disable it first");

        profile = newProfile;
        Close();
        if (!Open(current))
            return Report(ArmEventKind::targetFailed, id, "the adapter could not be opened");
        sink.OnEvent({ .kind = ArmEventKind::profileLoaded, .time = infra::Now(), .command = id, .commandName = "load profile" });
    }

    bool ArmRuntime::Open(const ArmTarget& target)
    {
        current = target;
        link = opener(target, profile.config.nodes);
        if (!link)
        {
            sink.OnSnapshot({ .target = target });
            return false;
        }

        session.emplace(link->Can(), profile, target, sink, link->Simulation());
        return true;
    }

    void ArmRuntime::Close()
    {
        session.reset();
        link.reset();
    }

    void ArmRuntime::FinishSwitch()
    {
        Close();
        switching = false;

        if (Open(switchTarget))
            return Report(ArmEventKind::targetReady, switchId);

        Report(ArmEventKind::targetFailed, switchId, "the adapter could not be opened");
        if (Open(previousTarget))
            Report(ArmEventKind::targetReady, 0);
    }

    void ArmRuntime::Report(ArmEventKind kind, uint32_t id, std::string_view reason)
    {
        sink.OnEvent({ .kind = kind, .time = infra::Now(), .command = id, .commandName = "switch target", .reason = reason });
    }
}
