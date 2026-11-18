#include "tools/arm_host/ArmRuntime.hpp"

namespace arm_host
{
    ArmRuntime::ArmRuntime(const arm::ArmProfile& profile, const TargetOpener& opener, ArmSessionSink& sink, const ArmTarget& initial)
        : profile(profile)
        , opener(opener)
        , sink(sink)
        , poll(pollInterval, [this]()
              {
                  if (!link)
                      return;
                  link->Poll();
                  ProceedConnecting();
              })
    {
        commissioning = initial.kind == TargetKind::real;
        if (!Open(initial, 0))
            Report(ArmEventKind::targetFailed, 0);
    }

    void ArmRuntime::Execute(uint32_t id, const ArmCommand& command)
    {
        if (switching && session && std::holds_alternative<EmergencyStop>(command))
            session->Execute(id, command);
        else if (switching)
            sink.OnEvent({ .kind = ArmEventKind::refused, .time = infra::Now(), .command = id, .commandName = CommandName(command), .reason = "switching target" });
        else if (connecting)
            sink.OnEvent({ .kind = ArmEventKind::refused, .time = infra::Now(), .command = id, .commandName = CommandName(command), .reason = "connecting" });
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
            sink.OnEvent({ .kind = ArmEventKind::refused, .time = infra::Now(), .command = id, .commandName = "load profile", .reason = std::string{ reason } });
        };

        if (switching)
            return refuse("switching target");
        if (session && session->Running())
            return refuse("the arm is enabled: disable it first");

        profile = newProfile;
        if (session && link->Adopt(profile.config))
        {
            session.reset();
            Build();
        }
        else
        {
            Close();
            if (!Open(current, std::nullopt))
                return Report(ArmEventKind::targetFailed, id, "the adapter could not be opened");
        }
        sink.OnEvent({ .kind = ArmEventKind::profileLoaded, .time = infra::Now(), .command = id, .commandName = "load profile" });
    }

    void ArmRuntime::SetCommissioning(uint32_t id, bool on)
    {
        const auto answer = [this, id](ArmEventKind kind, std::string_view reason)
        {
            sink.OnEvent({ .kind = kind, .time = infra::Now(), .command = id, .commandName = "commissioning limits", .reason = std::string{ reason } });
        };

        if (switching)
            return answer(ArmEventKind::refused, "switching target");
        if (current.kind != TargetKind::real || !session)
            return answer(ArmEventKind::refused, "not the real arm");
        if (session->Running())
            return answer(ArmEventKind::refused, "the arm is enabled: disable it first");

        commissioning = on;
        session.reset();
        Build();
        answer(ArmEventKind::accepted, "");
    }

    bool ArmRuntime::Open(const ArmTarget& target, std::optional<uint32_t> announce)
    {
        current = target;
        link = opener(target, profile.config);
        if (!link)
        {
            sink.OnSnapshot({ .target = target });
            return false;
        }

        connecting = Connecting{ announce, infra::Now() + link->ConnectTimeout() };
        connectingMotors.reset();
        ProceedConnecting();
        return true;
    }

    void ArmRuntime::ProceedConnecting()
    {
        if (!connecting)
            return;

        const auto announce = connecting->announce;
        const auto failure = link->Failure();
        if (link->Connected())
        {
            connecting.reset();
            Build();
            if (announce.has_value())
                Report(ArmEventKind::targetReady, *announce);
        }
        else if (failure.has_value() || infra::Now() >= connecting->deadline)
        {
            connecting.reset();
            link.reset();
            sink.OnSnapshot({ .target = current });
            Report(ArmEventKind::targetFailed, announce.value_or(0), failure.value_or("the adapter did not connect"));
        }
        else
            PublishConnecting();
    }

    void ArmRuntime::Build()
    {
        session.emplace(link->Can(), profile, current, sink, link->Simulation(), link->Motors(), commissioning && current.kind == TargetKind::real);
    }

    void ArmRuntime::PublishConnecting()
    {
        const auto* motors = link->Motors();
        if (motors == nullptr)
            return;

        auto status = motors->Status();
        if (connectingMotors == status)
            return;

        connectingMotors = status;
        sink.OnSnapshot({ .target = current, .virtualMotors = std::move(status) });
    }

    void ArmRuntime::Close()
    {
        connecting.reset();
        session.reset();
        link.reset();
    }

    void ArmRuntime::FinishSwitch()
    {
        Close();
        switching = false;

        commissioning = switchTarget.kind == TargetKind::real;
        if (Open(switchTarget, switchId))
            return;

        Report(ArmEventKind::targetFailed, switchId, "the adapter could not be opened");
        commissioning = previousTarget.kind == TargetKind::real;
        Open(previousTarget, 0);
    }

    void ArmRuntime::Report(ArmEventKind kind, uint32_t id, std::string reason)
    {
        sink.OnEvent({ .kind = kind, .time = infra::Now(), .command = id, .commandName = "switch target", .reason = std::move(reason) });
    }
}
