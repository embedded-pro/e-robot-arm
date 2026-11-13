#include "tools/arm_host/ArmSession.hpp"

namespace arm_host
{
    namespace
    {
        bool NeedsMotionPermission(const ArmCommand& command)
        {
            return std::holds_alternative<SelectMode>(command) || std::holds_alternative<StartJoints>(command) || std::holds_alternative<MoveTo>(command);
        }

        bool DeliveredByTheJoints(const ArmCommand& command)
        {
            return std::holds_alternative<CheckContracts>(command) || std::holds_alternative<SelectMode>(command) || std::holds_alternative<StartJoints>(command) ||
                   std::holds_alternative<StopJoints>(command) || std::holds_alternative<ClearFaults>(command);
        }

        bool LifecycleCommand(const ArmCommand& command)
        {
            return std::holds_alternative<BringUp>(command) || std::holds_alternative<Enable>(command) || std::holds_alternative<Disable>(command);
        }

        std::optional<arm::JointCommand> JointCommandOf(const ArmCommand& command)
        {
            if (std::holds_alternative<CheckContracts>(command))
                return arm::JointCommand::queryContracts;
            if (std::holds_alternative<SelectMode>(command))
                return arm::JointCommand::selectMode;
            if (std::holds_alternative<StartJoints>(command))
                return arm::JointCommand::start;
            if (std::holds_alternative<StopJoints>(command))
                return arm::JointCommand::stop;
            if (std::holds_alternative<ClearFaults>(command))
                return arm::JointCommand::clearFault;
            return std::nullopt;
        }

        std::string_view FailureReason(arm::CommandFailure failure)
        {
            switch (failure)
            {
                case arm::CommandFailure::timedOut:
                    return "not delivered to every joint in time";
                case arm::CommandFailure::replaced:
                    return "replaced by a newer command";
                case arm::CommandFailure::emergencyStop:
                    return "emergency stop";
            }

            return "not delivered";
        }
    }

    ArmSession::ArmSession(hal::Can& can, const arm::ArmProfile& profile, const ArmTarget& target, ArmSessionSink& sink, simulation::SimulatedArm* simulation)
        : profile(profile)
        , client(can)
        , arm(client.CategoryClient(), profile.config)
        , model(profile.config.geometry, profile.config.positionLimits)
        , target(target)
        , sink(sink)
        , simulation(simulation)
        , publishTimer(profile.config.controlPeriod, [this]()
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
        if (DeliveredByTheJoints(command))
            Deliver(id, command);
        else if (Run(command))
            Emit({ .kind = ArmEventKind::accepted, .command = id, .commandName = CommandName(command) });
        else
            Emit({ .kind = ArmEventKind::refused, .command = id, .commandName = CommandName(command), .reason = RefusalReason(command) });
    }

    void ArmSession::Deliver(uint32_t id, const ArmCommand& command)
    {
        issuing.emplace(id, command);
        const auto sent = Run(command);
        const auto deliveredAtOnce = !issuing.has_value();
        issuing.reset();

        if (!sent)
            Emit({ .kind = ArmEventKind::refused, .command = id, .commandName = CommandName(command), .reason = RefusalReason(command) });
        else if (!deliveredAtOnce)
            held.emplace(id, command);
    }

    void ArmSession::Delivered(arm::JointCommand command)
    {
        auto& delivered = issuing.has_value() ? issuing : held;
        if (!delivered.has_value() || JointCommandOf(delivered->command) != command)
            return;

        Emit({ .kind = ArmEventKind::accepted, .command = delivered->id, .commandName = CommandName(delivered->command) });
        delivered.reset();
    }

    void ArmSession::Failed(arm::JointCommand command, arm::CommandFailure failure)
    {
        if (!held.has_value() || JointCommandOf(held->command) != command)
            return;

        Emit({ .kind = ArmEventKind::refused, .command = held->id, .commandName = CommandName(held->command), .reason = FailureReason(failure) });
        held.reset();
    }

    void ArmSession::StopAll()
    {
        arm.Motion().Abort();
        arm.Safety().Stop();
        arm.Loop().Stop();
        publishTimer.Cancel();
        held.reset();
    }

    bool ArmSession::Moving() const
    {
        return arm.Motion().Moving();
    }

    bool ArmSession::Running() const
    {
        const auto state = arm.Lifecycle().State();
        return state == arm::ArmState::enabling || state == arm::ArmState::enabled || state == arm::ArmState::moving || state == arm::ArmState::disabling;
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
            const auto rearmed = session.arm.Lifecycle().Rearm();
            session.tripReported = session.tripReported && !rearmed;
            return rearmed;
        }

        bool operator()(const MoveTo& move) const
        {
            return session.arm.Lifecycle().MoveTo(move.target);
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

        bool operator()(const BringUp&) const
        {
            return session.arm.Lifecycle().BringUp();
        }

        bool operator()(const Enable&) const
        {
            return session.arm.Lifecycle().Enable();
        }

        bool operator()(const Disable&) const
        {
            return session.arm.Lifecycle().Disable();
        }

        bool operator()(const ApplyController& apply) const
        {
            return session.arm.Configurator().Apply(apply.joint, apply.settings);
        }

        bool operator()(const ReadController& read) const
        {
            return session.arm.Configurator().Read(read.joint);
        }

        bool operator()(const ApplyMotor& apply) const
        {
            return session.arm.Configurator().ApplyMotor(apply.joint, apply.settings);
        }

        bool operator()(const ReadMotor& read) const
        {
            return session.arm.Configurator().ReadMotor(read.joint);
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
            return arm.Lifecycle().State() == arm::ArmState::tripped ? "a joint is in fault" : "not tripped";
        if (LifecycleCommand(command))
            return StateName(arm.Lifecycle().State());
        if (std::holds_alternative<ApplyController>(command) || std::holds_alternative<ReadController>(command) || std::holds_alternative<ApplyMotor>(command) ||
            std::holds_alternative<ReadMotor>(command))
            return arm.Configurator().Busy() ? "busy" : "the arm is not ready";
        if (NeedsMotionPermission(command) && arm.Safety().Tripped())
            return "tripped";
        if (NeedsMotionPermission(command) && arm.Safety().Contracts() != can::ContractCompatibility::compatible)
            return "contracts not compatible";
        if (std::holds_alternative<MoveTo>(command) && arm.Lifecycle().State() != arm::ArmState::enabled)
            return Moving() ? "moving" : "the arm is not enabled";
        if (std::holds_alternative<MoveTo>(command))
            return "target outside the joint limits";
        if (DeliveredByTheJoints(command))
            return "a joint's contract is incompatible";
        return "CAN transmit queue full";
    }

    ArmSession::Events::Events(ArmSession& session, arm::Arm& arm)
        : arm::JointBusObserver(arm.Safety())
        , arm::JointMotionObserver(arm.Motion())
        , arm::ArmLifecycleObserver(arm.Lifecycle())
        , arm::JointConfiguratorObserver(arm.Configurator())
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

    void ArmSession::Events::OnCommandDelivered(arm::JointCommand command)
    {
        session.Delivered(command);
    }

    void ArmSession::Events::OnCommandFailed(arm::JointCommand command, arm::CommandFailure failure)
    {
        session.Failed(command, failure);
    }

    void ArmSession::Events::OnStateChanged(arm::ArmState state)
    {
        const auto misconfiguration = session.arm.Lifecycle().MisconfiguredJoint();
        if (state == arm::ArmState::misconfigured && misconfiguration.has_value())
            session.Emit({ .kind = ArmEventKind::stateChanged,
                .reason = MisconfigurationText(*misconfiguration),
                .joint = misconfiguration->joint,
                .state = state });
        else
            session.Emit({ .kind = ArmEventKind::stateChanged, .state = state });
    }

    void ArmSession::Events::OnApplied(std::size_t joint)
    {
        session.Emit({ .kind = ArmEventKind::controllerApplied, .joint = joint });
    }

    void ArmSession::Events::OnMotorApplied(std::size_t joint)
    {
        session.Emit({ .kind = ArmEventKind::motorApplied, .joint = joint });
    }

    void ArmSession::Events::OnMotorRead(std::size_t joint, const arm::MotorSettings& motor)
    {
        session.Emit({ .kind = ArmEventKind::motorRead, .joint = joint, .motor = motor });
    }

    void ArmSession::Events::OnRead(std::size_t joint, const arm::ControllerReading& reading)
    {
        session.Emit({ .kind = ArmEventKind::controllerRead, .joint = joint, .controller = reading });
    }

    void ArmSession::Events::OnConfigurationFailed(std::size_t joint, arm::ConfigurationFailure)
    {
        session.Emit({ .kind = ArmEventKind::controllerFailed, .reason = "no answer within 100 ms", .joint = joint });
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
        snapshot.state = arm.Lifecycle().State();
        snapshot.misconfiguration = arm.Lifecycle().MisconfiguredJoint();
        snapshot.overruns = arm.Loop().OverrunCount();
        arm::JointPositions measured{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            measured[joint] = snapshot.readings[joint].position;
        snapshot.toolPose = arm::ToPlacement(model.ToolPose(measured));
        snapshot.singularities = model.Near(measured);
        snapshot.profileName.assign(profile.name.begin(), profile.name.end());
        snapshot.config = profile.config;
        sample.commanded = snapshot.commanded.value_or(sample.measured);

        sink.OnSnapshot(snapshot);
        sink.OnTrace(sample);
    }
}
