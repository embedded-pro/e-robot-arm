#include "tools/arm_host/ArmSession.hpp"
#include "tools/arm_host/Commissioning.hpp"
#include <cmath>
#include <format>

namespace arm_host
{
    namespace
    {
        bool NeedsMotionPermission(const ArmCommand& command)
        {
            return std::holds_alternative<SelectMode>(command) || std::holds_alternative<StartJoints>(command) || std::holds_alternative<AlignJoints>(command) ||
                   std::holds_alternative<MoveTo>(command) || std::holds_alternative<StepTo>(command);
        }

        bool DeliveredByTheJoints(const ArmCommand& command)
        {
            return std::holds_alternative<CheckContracts>(command) || std::holds_alternative<SelectMode>(command) || std::holds_alternative<StartJoints>(command) ||
                   std::holds_alternative<StopJoints>(command) || std::holds_alternative<ClearFaults>(command) || std::holds_alternative<AlignJoints>(command);
        }

        std::optional<arm::ArmControllerId> ArmControllerOf(const ArmCommand& command)
        {
            if (const auto* enable = std::get_if<EnableArmController>(&command))
                return enable->id;
            if (const auto* disable = std::get_if<DisableArmController>(&command))
                return disable->id;
            if (const auto* select = std::get_if<SelectArmController>(&command))
                return select->id;
            return std::nullopt;
        }

        constexpr float maximumCollisionTorque{ 200.0f };

        bool TargetCommand(const ArmCommand& command)
        {
            return std::holds_alternative<InjectFault>(command) || std::holds_alternative<SilenceJoint>(command) ||
                   std::holds_alternative<ResumeJoint>(command) || std::holds_alternative<CollideJoint>(command) ||
                   std::holds_alternative<PowerOffMotor>(command) || std::holds_alternative<RestartMotor>(command);
        }

        bool Stopped(arm::ArmState state)
        {
            return state == arm::ArmState::tripped || state == arm::ArmState::holding;
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
            if (std::holds_alternative<AlignJoints>(command))
                return arm::JointCommand::align;
            return std::nullopt;
        }

        std::string SupervisionReason(arm::SupervisionTrip trip, float value)
        {
            switch (trip)
            {
                case arm::SupervisionTrip::trackingError:
                    return std::format("tracking error {:.3f} rad", value);
                case arm::SupervisionTrip::collision:
                    return std::format("collision {:.1f} N·m", value);
                case arm::SupervisionTrip::holdLost:
                    return std::format("hold lost: {:.3f} rad from the held pose", value);
                case arm::SupervisionTrip::speedLimit:
                    return std::format("speed {:.2f} rad/s above its limit", value);
                case arm::SupervisionTrip::referenceSpeed:
                    return std::format("reference speed {:.2f} rad/s above its motion limit", value);
                case arm::SupervisionTrip::referenceAcceleration:
                    return std::format("reference acceleration {:.2f} rad/s² above its motion limit", value);
                case arm::SupervisionTrip::referenceStep:
                    return std::format("reference jumped {:.3f} rad in one period", value);
                case arm::SupervisionTrip::workspace:
                case arm::SupervisionTrip::forbiddenZone:
                case arm::SupervisionTrip::selfCollision:
                    return std::format("workspace {:.3f} m", value);
                case arm::SupervisionTrip::positionLimit:
                    break;
            }

            return std::format("position {:.3f} rad outside its limits", value);
        }

        std::string LinkName(std::size_t link)
        {
            return link + 1 < arm::workspaceCapsules ? std::format("link {}", link + 1) : std::string{ "tool" };
        }

        std::string WorkspaceText(const arm::WorkspaceViolation& violation)
        {
            switch (violation.rule)
            {
                case arm::WorkspaceRule::workspace:
                    return std::format("{} outside the workspace by {:.3f} m", LinkName(violation.link), violation.distance);
                case arm::WorkspaceRule::forbiddenZone:
                    return std::format("{} in forbidden zone {} by {:.3f} m", LinkName(violation.link), violation.other + 1, violation.distance);
                case arm::WorkspaceRule::selfCollision:
                    break;
            }
            return std::format("{} and {} {:.3f} m apart (self-collision)", LinkName(violation.link), LinkName(violation.other), violation.distance);
        }

        std::string_view JointCommandName(arm::JointCommand command)
        {
            switch (command)
            {
                case arm::JointCommand::queryContracts:
                    return "check contracts";
                case arm::JointCommand::selectMode:
                    return "select mode";
                case arm::JointCommand::start:
                    return "start";
                case arm::JointCommand::stop:
                    return "stop";
                case arm::JointCommand::clearFault:
                    return "clear faults";
                case arm::JointCommand::align:
                    return "align";
            }

            return "joint command";
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

    ArmSession::ArmSession(hal::Can& can, const arm::ArmProfile& profile, const ArmTarget& target, ArmSessionSink& sink, simulation::SimulatedArm* simulation,
        VirtualMotors* virtualMotors, bool commissioning)
        : profile(profile)
        , commissioning(commissioning)
        , client(can)
        , arm(client.CategoryClient(), commissioning ? CommissioningConfig(profile.config) : profile.config,
              simulation != nullptr ? &simulation->HardwareStop() : nullptr)
        , model(profile.config.geometry, profile.config.mass, profile.config.positionLimits)
        , target(target)
        , sink(sink)
        , simulation(simulation)
        , virtualMotors(virtualMotors)
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
            Emit({ .kind = ArmEventKind::refused, .command = id, .commandName = CommandName(command), .reason = std::string{ RefusalReason(command) } });
    }

    void ArmSession::Deliver(uint32_t id, const ArmCommand& command)
    {
        issuing.emplace(id, command);
        const auto sent = Run(command);
        const auto deliveredAtOnce = !issuing.has_value();
        issuing.reset();

        if (!sent)
            Emit({ .kind = ArmEventKind::refused, .command = id, .commandName = CommandName(command), .reason = std::string{ RefusalReason(command) } });
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
        {
            Emit({ .kind = ArmEventKind::refused, .commandName = JointCommandName(command), .reason = std::string{ FailureReason(failure) } });
            return;
        }

        Emit({ .kind = ArmEventKind::refused, .command = held->id, .commandName = CommandName(held->command), .reason = std::string{ FailureReason(failure) } });
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

        bool operator()(const AlignJoints&) const
        {
            return session.arm.Safety().Align();
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

        bool operator()(const StepTo& step) const
        {
            return session.arm.Lifecycle().StepTo(step.target);
        }

        bool operator()(const AbortMove&) const
        {
            session.arm.Motion().Abort();
            return true;
        }

        bool operator()(const InjectFault& inject) const
        {
            return session.InjectFaultInto(inject);
        }

        bool operator()(const SilenceJoint& silence) const
        {
            return session.SetSilent(silence.joint, true);
        }

        bool operator()(const ResumeJoint& resume) const
        {
            return session.SetSilent(resume.joint, false);
        }

        bool operator()(const CollideJoint& collide) const
        {
            return session.Collide(collide);
        }

        bool operator()(const PowerOffMotor& powerOff) const
        {
            return session.PowerOff(powerOff.joint);
        }

        bool operator()(const RestartMotor& restart) const
        {
            return session.RestartVirtualMotor(restart.joint);
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

        bool operator()(const EnableArmController& enable) const
        {
            return session.arm.Controllers().Enable(enable.id);
        }

        bool operator()(const DisableArmController& disable) const
        {
            return session.arm.Controllers().Disable(disable.id);
        }

        bool operator()(const SelectArmController& select) const
        {
            return session.arm.Controllers().Select(select.id);
        }

        ArmSession& session;
    };

    bool ArmSession::Run(const ArmCommand& command)
    {
        return std::visit(Runner{ *this }, command);
    }

    bool ArmSession::Refuse(std::string_view reason)
    {
        refusal = reason;
        return false;
    }

    bool ArmSession::InjectFaultInto(const InjectFault& inject)
    {
        if (simulation == nullptr)
            return Refuse(virtualMotors != nullptr ? "not on the virtual arm" : "not a simulation");
        if (inject.joint >= arm::jointCount)
            return Refuse("no such joint");

        simulation->Joint(inject.joint).Fault(inject.fault);
        return true;
    }

    bool ArmSession::SetSilent(std::size_t joint, bool silent)
    {
        if (simulation == nullptr && virtualMotors == nullptr)
            return Refuse("not a simulation");
        if (joint >= arm::jointCount)
            return Refuse("no such joint");

        if (virtualMotors != nullptr)
            virtualMotors->Silence(joint, silent);
        else if (silent)
            simulation->Joint(joint).Silence();
        else
            simulation->Joint(joint).Resume();
        return true;
    }

    bool ArmSession::Collide(const CollideJoint& collide)
    {
        if (virtualMotors == nullptr)
            return Refuse(simulation != nullptr ? "not on simulated joints" : "not a simulation");
        if (collide.joint >= arm::jointCount)
            return Refuse("no such joint");
        if (!(std::abs(collide.torque.Value()) <= maximumCollisionTorque))
            return Refuse("torque outside ±200 N·m");

        virtualMotors->Collide(collide.joint, collide.torque);
        return true;
    }

    bool ArmSession::PowerOff(std::size_t joint)
    {
        if (virtualMotors == nullptr)
            return Refuse(simulation != nullptr ? "not on simulated joints" : "not a simulation");

        refusal = virtualMotors->PowerOff(joint).value_or("");
        return refusal.empty();
    }

    bool ArmSession::RestartVirtualMotor(std::size_t joint)
    {
        if (virtualMotors == nullptr)
            return Refuse("not a virtual arm");
        if (Running())
            return Refuse("disable the arm first");

        refusal = virtualMotors->Restart(joint).value_or("");
        return refusal.empty();
    }

    std::string_view ArmSession::RefusalReason(const ArmCommand& command) const
    {
        if (TargetCommand(command))
            return refusal;
        if (std::holds_alternative<DisableArmController>(command) && arm.Controllers().Active() == ArmControllerOf(command))
            return "the arm drives the joints with it: disable the arm first";
        if (const auto id = ArmControllerOf(command); id.has_value())
            return ArmControllerRefusal(*id);
        if (std::holds_alternative<Rearm>(command))
            return Stopped(arm.Lifecycle().State()) ? "a joint is in fault" : "not tripped or holding";
        if (LifecycleCommand(command))
            return StateName(arm.Lifecycle().State());
        if (std::holds_alternative<ApplyController>(command) || std::holds_alternative<ReadController>(command) || std::holds_alternative<ApplyMotor>(command) ||
            std::holds_alternative<ReadMotor>(command))
            return arm.Configurator().Busy() ? "busy" : "the arm is not ready";
        if (const auto reason = MotionPermissionRefusal(); NeedsMotionPermission(command) && reason.has_value())
            return *reason;
        if (std::holds_alternative<MoveTo>(command) || std::holds_alternative<StepTo>(command))
            return MoveRefusal(command);
        if (DeliveredByTheJoints(command))
            return "a joint's contract is incompatible";
        return "CAN transmit queue full";
    }

    std::optional<std::string_view> ArmSession::MotionPermissionRefusal() const
    {
        if (arm.Safety().Tripped())
            return "tripped";
        if (arm.Safety().Holding())
            return "holding after a protective stop: re-arm first";
        if (arm.Safety().Contracts() != can::ContractCompatibility::compatible)
            return "contracts not compatible";
        return std::nullopt;
    }

    std::string_view ArmSession::MoveRefusal(const ArmCommand& command) const
    {
        if (arm.Lifecycle().State() == arm::ArmState::holding)
            return "holding after a protective stop: re-arm first";
        if (arm.Lifecycle().State() != arm::ArmState::enabled)
            return Moving() ? "moving" : "the arm is not enabled";
        const auto& target = std::holds_alternative<MoveTo>(command) ? std::get<MoveTo>(command).target : std::get<StepTo>(command).target;
        if (arm::WithinLimits(profile.config.positionLimits, target))
        {
            if (const auto violation = arm.Workspace().Check(target); violation.has_value())
            {
                workspaceRefusal = "target: " + WorkspaceText(*violation);
                return workspaceRefusal;
            }
            if (std::holds_alternative<MoveTo>(command))
                return "its path leaves the workspace, enters a forbidden zone or meets the arm itself";
        }
        if (std::holds_alternative<MoveTo>(command))
            return "target outside the joint limits";
        return "a step is at most 0.2 rad from where the joints are, within the joint limits";
    }

    std::string_view ArmSession::ArmControllerRefusal(arm::ArmControllerId id) const
    {
        const auto& controllers = arm.Controllers();
        if (id >= controllers.Count())
            return "no such arm controller";
        if (!controllers.Enabled(id))
            return "the arm controller is disabled";
        return "a move runs and the arm controller cannot take over without a step";
    }

    ArmSession::Events::Events(ArmSession& session, arm::Arm& arm)
        : arm::JointBusObserver(arm.Safety())
        , arm::JointMotionObserver(arm.Motion())
        , arm::ArmLifecycleObserver(arm.Lifecycle())
        , arm::JointConfiguratorObserver(arm.Configurator())
        , arm::ArmControllersObserver(arm.Controllers())
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

    void ArmSession::Events::OnSupervisionTrip(std::size_t joint, arm::SupervisionTrip trip, float value)
    {
        auto reason = SupervisionReason(trip, value);
        session.Emit({ .kind = ArmEventKind::supervisionTrip, .reason = reason, .joint = joint });
        session.ReportTrip(joint, can::FocFaultCode::none, std::move(reason));
    }

    void ArmSession::Events::OnHardwareStop(bool open)
    {
        session.Emit({ .kind = ArmEventKind::hardwareStop, .reason = open ? "hardware stop open" : "hardware stop closed" });
        if (open)
            session.ReportTrip(arm_host::noJoint, can::FocFaultCode::none, "hardware stop open");
    }

    void ArmSession::Events::OnProtectiveStop(std::size_t joint, arm::SupervisionTrip trip, float value)
    {
        const auto workspace = session.arm.Safety().LastWorkspaceViolation();
        const bool posed = trip == arm::SupervisionTrip::workspace || trip == arm::SupervisionTrip::forbiddenZone || trip == arm::SupervisionTrip::selfCollision;
        auto reason = posed && workspace.has_value() ? WorkspaceText(*workspace) : SupervisionReason(trip, value);
        session.Emit({ .kind = ArmEventKind::protectiveStop, .reason = std::move(reason), .joint = joint });
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
                .reason = std::string{ MisconfigurationText(*misconfiguration) },
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

    void ArmSession::Events::OnWatchdogApplied(std::size_t)
    {}

    void ArmSession::Events::OnWatchdogRead(std::size_t, float)
    {}

    void ArmSession::Events::OnRead(std::size_t joint, const arm::ControllerReading& reading)
    {
        session.Emit({ .kind = ArmEventKind::controllerRead, .joint = joint, .controller = reading });
    }

    void ArmSession::Events::OnConfigurationFailed(std::size_t joint, arm::ConfigurationFailure)
    {
        const auto window = std::chrono::duration_cast<std::chrono::milliseconds>(arm::SilenceWindow(session.profile.config)).count();
        session.Emit({ .kind = ArmEventKind::controllerFailed, .reason = std::format("no answer within {} ms", window), .joint = joint });
    }

    void ArmSession::Events::OnActiveControllerChanged(std::optional<arm::ArmControllerId> active)
    {
        const auto name = active.has_value() ? session.arm.Controllers().Name(*active) : infra::BoundedConstString{ "none" };
        session.Emit({ .kind = ArmEventKind::armControllerChanged, .armController = std::string_view{ name.data(), name.size() } });
    }

    void ArmSession::Events::OnMoveDone()
    {
        session.Emit({ .kind = ArmEventKind::moveDone });
    }

    void ArmSession::Events::OnMoveAborted()
    {
        session.Emit({ .kind = ArmEventKind::moveAborted });
    }

    void ArmSession::ReportTrip(std::size_t joint, can::FocFaultCode fault, std::string reason)
    {
        if (tripReported || !arm.Safety().Tripped())
            return;

        tripReported = true;
        Emit({ .kind = ArmEventKind::tripped, .reason = std::move(reason), .joint = joint, .fault = fault });
    }

    void ArmSession::Emit(ArmEvent event)
    {
        event.time = infra::Now();
        sink.OnEvent(event);
    }

    void ArmSession::PublishDisturbances(ArmSnapshot& snapshot) const
    {
        if (simulation != nullptr)
        {
            Disturbances disturbances{ .injectFault = true };
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                disturbances.silent[joint] = simulation->Joint(joint).Silent();
            snapshot.disturbances = disturbances;
        }
        else if (virtualMotors != nullptr)
        {
            snapshot.virtualMotors = virtualMotors->Status();
            snapshot.disturbances =
                Disturbances{ .collision = true, .powerOff = true, .silent = snapshot.virtualMotors->silenced, .torque = snapshot.virtualMotors->torques };
        }
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

        if (const auto setpoints = arm.Controllers().LastSetpoints())
            snapshot.commanded = setpoints->positions;
        else
            snapshot.commanded.reset();
        snapshot.externalTorques = arm.EstimatedExternalTorques();
        sample.externalTorques = snapshot.externalTorques;
        if (const auto setpoints = arm.Controllers().LastSetpoints())
            sample.torques = setpoints->torques;
        snapshot.contracts = safety.Contracts();
        snapshot.tripped = safety.Tripped();
        snapshot.holding = safety.Holding();
        snapshot.hardwareStopOpen = safety.HardwareStopOpen();
        snapshot.torqueClamped = safety.TorqueClamped();
        snapshot.moving = arm.Motion().Moving();
        snapshot.state = arm.Lifecycle().State();
        snapshot.misconfiguration = arm.Lifecycle().MisconfiguredJoint();
        snapshot.overruns = arm.Loop().OverrunCount();
        arm::JointPositions measured{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            measured[joint] = snapshot.readings[joint].position;
        snapshot.toolPose = arm::ToPlacement(model.ToolPose(measured));
        snapshot.singularities = model.Near(measured);
        snapshot.gravityTorque = model.GravityTorque(measured);
        snapshot.manipulability = model.Manipulability(measured);
        snapshot.profileName.assign(profile.name.begin(), profile.name.end());
        snapshot.config = profile.config;
        snapshot.commissioning = commissioning;
        const auto& controllers = arm.Controllers();
        for (arm::ArmControllerId id = 0; id != controllers.Count(); ++id)
        {
            const auto name = controllers.Name(id);
            snapshot.armControllers.emplace_back(std::string{ name.data(), name.size() }, controllers.Enabled(id), controllers.Active() == id);
        }
        sample.commanded = snapshot.commanded.value_or(sample.measured);
        PublishDisturbances(snapshot);

        sink.OnSnapshot(snapshot);
        sink.OnTrace(sample);
    }
}
