#include "tools/arm_host/ArmSession.hpp"

namespace arm_host
{
    namespace
    {
        bool NeedsMotionPermission(const ArmCommand& command)
        {
            return std::holds_alternative<SelectMode>(command) || std::holds_alternative<StartJoints>(command) || std::holds_alternative<MoveTo>(command);
        }

        bool IdempotentBusCommand(const ArmCommand& command)
        {
            return std::holds_alternative<CheckContracts>(command) || std::holds_alternative<SelectMode>(command) || std::holds_alternative<StartJoints>(command) ||
                   std::holds_alternative<StopJoints>(command) || std::holds_alternative<ClearFaults>(command) || std::holds_alternative<EmergencyStop>(command);
        }
    }

    ArmSession::ArmSession(hal::Can& can, const arm::ArmConfig& config, const ArmTarget& target, ArmSessionSink& sink, simulation::SimulatedArm* simulation)
        : client(can)
        , arm(client.CategoryClient(), config)
        , target(target)
        , sink(sink)
        , simulation(simulation)
        , publishTimer(config.controlPeriod, [this]()
              {
                  Publish();
              })
    {
        arm.Loop().Start();
        Publish();
    }

    ArmSession::~ArmSession()
    {
        arm.Loop().Stop();
    }

    void ArmSession::Execute(uint32_t id, const ArmCommand& command)
    {
        if (std::holds_alternative<EmergencyStop>(command))
        {
            backlog.push_front({ id, command });
            attempts = 0;
        }
        else
            backlog.push_back({ id, command });

        if (!retryTimer.Armed())
            Pump();
    }

    void ArmSession::Pump()
    {
        while (!backlog.empty())
        {
            const auto& [id, command] = backlog.front();
            if (Run(command))
                Emit({ .kind = ArmEventKind::accepted, .command = id, .commandName = CommandName(command) });
            else if (QueueRefused(command) && ++attempts < maxAttempts)
            {
                retryTimer.Start(retryInterval, [this]()
                    {
                        Pump();
                    });
                return;
            }
            else
                Emit({ .kind = ArmEventKind::refused, .command = id, .commandName = CommandName(command), .reason = RefusalReason(command) });

            attempts = 0;
            backlog.pop_front();
        }
    }

    bool ArmSession::QueueRefused(const ArmCommand& command) const
    {
        return IdempotentBusCommand(command) && RefusalReason(command) == "CAN transmit queue full";
    }

    void ArmSession::StopAll()
    {
        arm.Motion().Abort();
        arm.Safety().Stop();
        arm.Loop().Stop();
        publishTimer.Cancel();
        retryTimer.Cancel();
        backlog.clear();
    }

    bool ArmSession::Moving() const
    {
        return arm.Motion().Moving();
    }

    struct ArmSession::Runner
    {
        bool operator()(const CheckContracts&) const
        {
            return session.arm.Safety().QueryContracts();
        }

        bool operator()(const SelectMode& select) const
        {
            return session.arm.Safety().SelectMode(select.mode);
        }

        bool operator()(const StartJoints&) const
        {
            return session.arm.Safety().Start();
        }

        bool operator()(const StopJoints&) const
        {
            return session.arm.Safety().Stop();
        }

        bool operator()(const ClearFaults&) const
        {
            return session.arm.Safety().ClearFault();
        }

        bool operator()(const EmergencyStop&) const
        {
            session.arm.Motion().Abort();
            return session.arm.Safety().EmergencyStop();
        }

        bool operator()(const Rearm&) const
        {
            const auto rearmed = session.arm.Safety().Rearm();
            session.tripReported = session.tripReported && !rearmed;
            return rearmed;
        }

        bool operator()(const MoveTo& move) const
        {
            return session.arm.Motion().MoveTo(move.target);
        }

        bool operator()(const AbortMove&) const
        {
            session.arm.Motion().Abort();
            return true;
        }

        bool operator()(const InjectFault& inject) const
        {
            return session.Simulate(inject.joint, [&inject](simulation::SimulatedJoint& joint)
                {
                    joint.Fault(inject.fault);
                });
        }

        bool operator()(const SilenceJoint& silence) const
        {
            return session.Simulate(silence.joint, [](simulation::SimulatedJoint& joint)
                {
                    joint.Silence();
                });
        }

        ArmSession& session;
    };

    bool ArmSession::Run(const ArmCommand& command)
    {
        return std::visit(Runner{ *this }, command);
    }

    bool ArmSession::Simulate(std::size_t joint, const infra::Function<void(simulation::SimulatedJoint&)>& action)
    {
        if (simulation == nullptr || joint >= arm::jointCount)
            return false;

        action(simulation->Joint(joint));
        return true;
    }

    std::string_view ArmSession::RefusalReason(const ArmCommand& command) const
    {
        if (std::holds_alternative<InjectFault>(command) || std::holds_alternative<SilenceJoint>(command))
            return simulation == nullptr ? "not a simulated target" : "no such joint";
        if (std::holds_alternative<Rearm>(command))
            return "a joint is in fault";
        if (NeedsMotionPermission(command) && arm.Safety().Tripped())
            return "tripped";
        if (NeedsMotionPermission(command) && arm.Safety().Contracts() != can::ContractCompatibility::compatible)
            return "contracts not compatible";
        if (std::holds_alternative<MoveTo>(command))
            return Moving() ? "moving" : "target outside the joint limits";
        return "CAN transmit queue full";
    }

    ArmSession::Events::Events(ArmSession& session, arm::Arm& arm)
        : arm::JointBusObserver(arm.Safety())
        , arm::JointMotionObserver(arm.Motion())
        , session(session)
    {}

    void ArmSession::Events::OnContractsResolved(can::ContractCompatibility compatibility)
    {
        session.Emit({ .kind = ArmEventKind::contractsResolved, .contracts = compatibility });
    }

    void ArmSession::Events::OnJointFault(std::size_t joint, can::FocFaultCode fault)
    {
        session.Emit({ .kind = ArmEventKind::jointFault, .joint = joint, .fault = fault });
        session.ReportTrip(joint, fault);
    }

    void ArmSession::Events::OnJointSilent(std::size_t joint)
    {
        session.Emit({ .kind = ArmEventKind::jointSilent, .joint = joint });
        session.ReportTrip(joint, can::FocFaultCode::none);
    }

    void ArmSession::Events::OnMoveDone()
    {
        session.Emit({ .kind = ArmEventKind::moveDone });
    }

    void ArmSession::Events::OnMoveAborted()
    {
        session.Emit({ .kind = ArmEventKind::moveAborted });
    }

    void ArmSession::ReportTrip(std::size_t joint, can::FocFaultCode fault)
    {
        if (tripReported || !arm.Safety().Tripped())
            return;

        tripReported = true;
        Emit({ .kind = ArmEventKind::tripped, .joint = joint, .fault = fault });
    }

    void ArmSession::Emit(ArmEvent event)
    {
        event.time = infra::Now();
        sink.OnEvent(event);
    }

    void ArmSession::Publish()
    {
        const auto& safety = arm.Safety();

        ArmSnapshot snapshot{ .target = target, .targetReady = true };
        TraceSample sample{ .time = infra::Now() };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            snapshot.readings[joint] = safety.Reading(joint);
            sample.measured[joint] = snapshot.readings[joint].position;
        }

        snapshot.commanded = arm.Motion().Setpoint();
        snapshot.contracts = safety.Contracts();
        snapshot.tripped = safety.Tripped();
        snapshot.moving = arm.Motion().Moving();
        sample.commanded = snapshot.commanded.value_or(sample.measured);

        sink.OnSnapshot(snapshot);
        sink.OnTrace(sample);
    }
}
