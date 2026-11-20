#include "core/lifecycle/implementations/ArmLifecycleImpl.hpp"
#include "numerical/math/Math.hpp"
#include "services/fsm/StateMachineDefinition.hpp"
#include <algorithm>

namespace arm
{
    using namespace lifecycle;

    namespace
    {
        bool IsCompatible(const ContractsResolved& event)
        {
            return event.compatibility == can::ContractCompatibility::compatible;
        }

        bool IsIncompatible(const ContractsResolved& event)
        {
            return event.compatibility == can::ContractCompatibility::incompatible;
        }

        constexpr std::size_t configurationSteps{ 3 * jointCount };

        bool Calibrated(can::FocMotorState state)
        {
            return state == can::FocMotorState::ready || state == can::FocMotorState::running;
        }
    }

    constexpr std::array<ArmLifecycleImpl::Machine::Transition, ArmLifecycleImpl::rowCount> ArmLifecycleImpl::Rows()
    {
        constexpr auto compatible = [](ArmLifecycleImpl&, const auto&, const ContractsResolved& event)
        {
            return IsCompatible(event);
        };
        constexpr auto incompatible = [](ArmLifecycleImpl&, const auto&, const ContractsResolved& event)
        {
            return IsIncompatible(event);
        };
        constexpr auto queryContracts = [](ArmLifecycleImpl& self, auto&, const lifecycle::BringUp&)
        {
            self.QueryContracts();
            return Checking{};
        };

        return { {
            Machine::Row<Unpowered, lifecycle::BringUp, Checking>(nullptr, queryContracts),
            Machine::Row<Incompatible, lifecycle::BringUp, Checking>(nullptr, queryContracts),
            Machine::Row<lifecycle::Misconfigured, lifecycle::BringUp, Checking>(nullptr, queryContracts),

            Machine::Row<Unpowered, ContractsResolved, WaitingForCalibration>(compatible),
            Machine::Row<Unpowered, ContractsResolved, lifecycle::Incompatible>(incompatible),
            Machine::Row<Checking, ContractsResolved, WaitingForCalibration>(compatible),
            Machine::Row<Checking, ContractsResolved, lifecycle::Incompatible>(incompatible),
            Machine::Row<lifecycle::Incompatible, ContractsResolved, WaitingForCalibration>(compatible),
            Machine::Row<WaitingForCalibration, ContractsResolved, lifecycle::Incompatible>(incompatible),
            Machine::Row<Ready, ContractsResolved, lifecycle::Incompatible>(incompatible),
            Machine::Row<Configuring, ContractsResolved, lifecycle::Incompatible>(incompatible),
            Machine::Row<lifecycle::Misconfigured, ContractsResolved, lifecycle::Incompatible>(incompatible),

            Machine::Row<Checking, Failed, Unpowered>([](ArmLifecycleImpl&, const Checking&, const Failed& event)
                {
                    return event.command == JointCommand::queryContracts;
                }),

            Machine::Row<WaitingForCalibration, Observed, Ready>([](ArmLifecycleImpl& self, const WaitingForCalibration&, const Observed& event)
                {
                    return event.allCalibrated && !self.HasConfiguration();
                }),
            Machine::Row<WaitingForCalibration, Observed, Configuring>([](ArmLifecycleImpl&, const WaitingForCalibration&, const Observed& event)
                {
                    return event.allCalibrated;
                },
                [](ArmLifecycleImpl& self, WaitingForCalibration&, const Observed&)
                {
                    self.StartConfiguring();
                    return Configuring{};
                }),
            Machine::Row<Configuring, Observed, WaitingForCalibration>([](ArmLifecycleImpl&, const Configuring&, const Observed& event)
                {
                    return !event.allCalibrated;
                }),
            Machine::InternalRow<Configuring, Observed>([](ArmLifecycleImpl& self, Configuring&, const Observed&)
                {
                    self.ConfigureNext();
                }),
            Machine::Row<Configuring, Configured, Ready>(),
            Machine::Row<Configuring, lifecycle::Misconfigure, lifecycle::Misconfigured>(nullptr, [](ArmLifecycleImpl& self, Configuring&, const lifecycle::Misconfigure& event)
                {
                    self.misconfiguration = event.misconfiguration;
                    return lifecycle::Misconfigured{};
                }),
            Machine::Row<Ready, Observed, Enabled>([](ArmLifecycleImpl&, const Ready&, const Observed& event)
                {
                    return event.allRunning;
                }),
            Machine::Row<Ready, Observed, WaitingForCalibration>([](ArmLifecycleImpl&, const Ready&, const Observed& event)
                {
                    return !event.allCalibrated;
                }),

            Machine::Row<Ready, lifecycle::Enable, Enabling>(nullptr, [](ArmLifecycleImpl& self, Ready&, const lifecycle::Enable&)
                {
                    self.unstartedJoint.reset();
                    self.SelectMode();
                    return Enabling{};
                }),
            Machine::InternalRow<Enabling, Delivered>([](ArmLifecycleImpl& self, Enabling& enabling, const Delivered&)
                {
                    enabling.started = true;
                    self.StartJoints();
                },
                [](ArmLifecycleImpl&, const Enabling&, const Delivered& event)
                {
                    return event.command == JointCommand::selectMode;
                }),
            Machine::Row<Enabling, Failed, Ready>([](ArmLifecycleImpl&, const Enabling&, const Failed& event)
                {
                    return event.command == JointCommand::selectMode || event.command == JointCommand::start;
                }),
            Machine::Row<Enabling, Observed, Enabled>([](ArmLifecycleImpl&, const Enabling&, const Observed& event)
                {
                    return event.allRunning;
                }),
            Machine::Row<Enabling, Observed, Disabling>([](ArmLifecycleImpl&, const Enabling& enabling, const Observed&)
                {
                    return enabling.started && enabling.periodsStarting + 1 >= giveUpPeriods;
                },
                [](ArmLifecycleImpl& self, Enabling&, const Observed&)
                {
                    self.GiveUpStarting();
                    return Disabling{};
                }),
            Machine::InternalRow<Enabling, Observed>([](ArmLifecycleImpl& self, Enabling& enabling, const Observed&)
                {
                    self.KeepStarting(enabling);
                }),

            Machine::Row<Enabled, Observed, Ready>([](ArmLifecycleImpl&, const Enabled&, const Observed& event)
                {
                    return !event.allRunning;
                }),
            Machine::Row<Enabled, MoveStarted, Moving>(),
            Machine::Row<Moving, MoveFinished, Enabled>(),
            Machine::Row<Moving, Observed, Ready>([](ArmLifecycleImpl&, const Moving&, const Observed& event)
                {
                    return !event.allRunning;
                },
                [](ArmLifecycleImpl& self, Moving&, const Observed&)
                {
                    self.motion.Abort();
                    return Ready{};
                }),

            Machine::Row<Enabled, lifecycle::Disable, Disabling>(nullptr, [](ArmLifecycleImpl& self, Enabled&, const lifecycle::Disable&)
                {
                    self.StopJoints();
                    return Disabling{};
                }),
            Machine::Row<Disabling, Observed, Ready>([](ArmLifecycleImpl&, const Disabling&, const Observed& event)
                {
                    return !event.anyRunning;
                }),
            Machine::Row<Disabling, Failed, Tripped>([](ArmLifecycleImpl&, const Disabling&, const Failed& event)
                {
                    return event.command == JointCommand::stop;
                },
                [](ArmLifecycleImpl& self, Disabling&, const Failed&)
                {
                    self.safety.EmergencyStop();
                    return Tripped{};
                }),

            Machine::Row<Enabled, ProtectiveStop, Holding>(),
            Machine::Row<Moving, ProtectiveStop, Holding>(nullptr, [](ArmLifecycleImpl& self, Moving&, const ProtectiveStop&)
                {
                    self.motion.Abort();
                    return Holding{};
                }),
            Machine::Row<Holding, Rearmed, Enabled>(),
            Machine::Row<Holding, lifecycle::Disable, Disabling>(nullptr, [](ArmLifecycleImpl& self, Holding&, const lifecycle::Disable&)
                {
                    self.StopJoints();
                    return Disabling{};
                }),
            Machine::Row<Holding, Observed, Ready>([](ArmLifecycleImpl&, const Holding&, const Observed& event)
                {
                    return !event.allRunning;
                }),

            Machine::InternalRow<Tripped, Trip>(),
            Machine::RowFromAny<Trip, Tripped>(nullptr, [](ArmLifecycleImpl& self, const lifecycle::State& from, const Trip&)
                {
                    if (std::holds_alternative<Moving>(from))
                        self.motion.Abort();
                    return Tripped{};
                }),
            Machine::Row<Tripped, Rearmed, WaitingForCalibration>(),
        } };
    }

    constexpr ArmLifecycleImpl::Machine::Rules ArmLifecycleImpl::Rules()
    {
        return Machine::Rules{}
            .Allow<Unpowered, Checking, WaitingForCalibration, lifecycle::Incompatible, Tripped>()
            .Allow<Checking, WaitingForCalibration, lifecycle::Incompatible, Unpowered, Tripped>()
            .Allow<lifecycle::Incompatible, Checking, WaitingForCalibration, Tripped>()
            .Allow<WaitingForCalibration, Ready, Configuring, lifecycle::Incompatible, Tripped>()
            .Allow<Configuring, Ready, lifecycle::Misconfigured, WaitingForCalibration, lifecycle::Incompatible, Tripped>()
            .Allow<lifecycle::Misconfigured, Checking, lifecycle::Incompatible, Tripped>()
            .Allow<Ready, Enabled, WaitingForCalibration, Enabling, lifecycle::Incompatible, Tripped>()
            .Allow<Enabling, Ready, Enabled, Disabling, Tripped>()
            .Allow<Enabled, Ready, Moving, Disabling, Holding, Tripped>()
            .Allow<Moving, Enabled, Ready, Holding, Tripped>()
            .Allow<Holding, Enabled, Disabling, Ready, Tripped>()
            .Allow<Disabling, Ready, Tripped>()
            .Allow<Tripped, WaitingForCalibration>();
    }

    ArmLifecycleImpl::ArmLifecycleImpl(SafetySupervisor& safety, JointMotion& motion, JointConfigurator& configurator, const JointControllers& controllers, const JointMotors& motors,
        std::optional<float> watchdogSeconds)
        : JointBusObserver(safety)
        , JointMotionObserver(motion)
        , JointConfiguratorObserver(configurator)
        , safety(safety)
        , motion(motion)
        , configurator(configurator)
        , controllers(controllers)
        , motors(motors)
        , watchdogSeconds(watchdogSeconds)
        , fsm(*this, services::Validated<ArmLifecycleImpl>())
    {
        fsm.Start<Unpowered>();
    }

    bool ArmLifecycleImpl::BringUp()
    {
        return Accepted(lifecycle::BringUp{});
    }

    bool ArmLifecycleImpl::Enable()
    {
        return Accepted(lifecycle::Enable{});
    }

    bool ArmLifecycleImpl::Disable()
    {
        return Accepted(lifecycle::Disable{});
    }

    bool ArmLifecycleImpl::Rearm()
    {
        return (fsm.Is<Tripped>() || fsm.Is<Holding>()) && safety.Rearm() && Accepted(Rearmed{});
    }

    bool ArmLifecycleImpl::MoveTo(const JointPositions& target)
    {
        if (fsm.Is<Moving>())
            return motion.MoveTo(target);
        return fsm.Is<Enabled>() && motion.MoveTo(target) && Accepted(MoveStarted{});
    }

    bool ArmLifecycleImpl::StepTo(const JointPositions& target)
    {
        return fsm.Is<Enabled>() && motion.StepTo(target) && Accepted(MoveStarted{});
    }

    bool ArmLifecycleImpl::MoveThrough(const JointWaypoints& waypoints)
    {
        return fsm.Is<Enabled>() && motion.MoveThrough(waypoints) && Accepted(MoveStarted{});
    }

    bool ArmLifecycleImpl::Jog(const JointJog& jog)
    {
        if (fsm.Is<Moving>())
            return motion.Jog(jog);
        return fsm.Is<Enabled>() && motion.Jog(jog) && Accepted(MoveStarted{});
    }

    bool ArmLifecycleImpl::Jog(const ToolJog& jog)
    {
        if (fsm.Is<Moving>())
            return motion.Jog(jog);
        return fsm.Is<Enabled>() && motion.Jog(jog) && Accepted(MoveStarted{});
    }

    bool ArmLifecycleImpl::MoveLine(const ToolPose& target)
    {
        return fsm.Is<Enabled>() && motion.MoveLine(target) && Accepted(MoveStarted{});
    }

    ArmState ArmLifecycleImpl::State() const
    {
        return static_cast<ArmState>(fsm.CurrentState().index());
    }

    std::optional<Misconfiguration> ArmLifecycleImpl::MisconfiguredJoint() const
    {
        if (fsm.Is<lifecycle::Misconfigured>())
            return misconfiguration;
        return std::nullopt;
    }

    std::optional<std::size_t> ArmLifecycleImpl::UnstartedJoint() const
    {
        return unstartedJoint;
    }

    void ArmLifecycleImpl::Step()
    {
        fsm.Dispatch(Observe());
    }

    void ArmLifecycleImpl::OnContractsResolved(can::ContractCompatibility compatibility)
    {
        fsm.Dispatch(ContractsResolved{ compatibility });
    }

    void ArmLifecycleImpl::OnJointFault(std::size_t, can::FocFaultCode)
    {
        fsm.Dispatch(Trip{});
    }

    void ArmLifecycleImpl::OnJointSilent(std::size_t)
    {
        fsm.Dispatch(Trip{});
    }

    void ArmLifecycleImpl::OnSupervisionTrip(std::size_t, SupervisionTrip, float)
    {
        fsm.Dispatch(Trip{});
    }

    void ArmLifecycleImpl::OnHardwareStop(bool open)
    {
        if (open)
            fsm.Dispatch(Trip{});
    }

    void ArmLifecycleImpl::OnProtectiveStop(std::size_t, SupervisionTrip, float)
    {
        fsm.Dispatch(ProtectiveStop{});
    }

    void ArmLifecycleImpl::OnCommandDelivered(JointCommand command)
    {
        fsm.Dispatch(Delivered{ command });
    }

    void ArmLifecycleImpl::OnCommandFailed(JointCommand command, CommandFailure)
    {
        fsm.Dispatch(Failed{ command });
    }

    void ArmLifecycleImpl::OnMoveDone()
    {
        fsm.Dispatch(MoveFinished{});
    }

    void ArmLifecycleImpl::OnMoveAborted()
    {
        fsm.Dispatch(MoveFinished{});
    }

    void ArmLifecycleImpl::OnApplied(std::size_t joint)
    {
        if (WorkingOn(joint, ConfigurationPart::controller) && !configurator.Read(joint))
            Misconfigure(MisconfigurationReason::failed);
    }

    void ArmLifecycleImpl::OnRead(std::size_t joint, const ControllerReading& reading)
    {
        if (!WorkingOn(joint, ConfigurationPart::controller))
            return;

        if (Compare(*controllers[joint], reading.settings).Any())
            Misconfigure(MisconfigurationReason::differs);
        else
            Verified();
    }

    void ArmLifecycleImpl::OnMotorApplied(std::size_t joint)
    {
        if (WorkingOn(joint, ConfigurationPart::motor) && !configurator.ReadMotor(joint))
            Misconfigure(MisconfigurationReason::failed);
    }

    void ArmLifecycleImpl::OnMotorRead(std::size_t joint, const MotorSettings& motor)
    {
        if (!WorkingOn(joint, ConfigurationPart::motor))
            return;

        if (Compare(motors[joint]->settings, motor).Any())
            Misconfigure(MisconfigurationReason::differs);
        else
            Verified();
    }

    void ArmLifecycleImpl::OnWatchdogApplied(std::size_t joint)
    {
        if (WorkingOn(joint, ConfigurationPart::watchdog) && !configurator.ReadWatchdog(joint))
            Misconfigure(MisconfigurationReason::failed);
    }

    void ArmLifecycleImpl::OnWatchdogRead(std::size_t joint, float timeoutSeconds)
    {
        if (!WorkingOn(joint, ConfigurationPart::watchdog))
            return;

        if (math::Abs(timeoutSeconds - *watchdogSeconds) > 1.0e-4f * math::Abs(*watchdogSeconds))
            Misconfigure(MisconfigurationReason::differs);
        else
            Verified();
    }

    void ArmLifecycleImpl::OnConfigurationFailed(std::size_t joint, ConfigurationFailure)
    {
        if (fsm.Is<Configuring>() && configuringStep.has_value() && JointOf(*configuringStep) == joint)
            Misconfigure(MisconfigurationReason::failed);
    }

    ArmLifecycleImpl::StateReporter::StateReporter(ArmLifecycleImpl& lifecycle, Machine& machine)
        : services::StateMachineObserver<lifecycle::State, lifecycle::Event>(machine)
        , lifecycle(lifecycle)
    {}

    void ArmLifecycleImpl::StateReporter::StateChanged(StateId, const lifecycle::Event&, StateId to)
    {
        const auto state = static_cast<ArmState>(to.Index());
        lifecycle.NotifyObservers([state](auto& observer)
            {
                observer.OnStateChanged(state);
            });
    }

    bool ArmLifecycleImpl::Accepted(const lifecycle::Event& event)
    {
        return fsm.Dispatch(event) == services::DispatchResult::transitioned;
    }

    Observed ArmLifecycleImpl::Observe() const
    {
        Observed observed{ true, true, false };
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto state = safety.Reading(joint).state;
            const auto running = state == can::FocMotorState::running;
            observed.allCalibrated = observed.allCalibrated && Calibrated(state);
            observed.allRunning = observed.allRunning && running;
            observed.anyRunning = observed.anyRunning || running;
        }

        return observed;
    }

    void ArmLifecycleImpl::QueryContracts()
    {
        if (!safety.QueryContracts())
            fsm.Dispatch(Failed{ JointCommand::queryContracts });
    }

    void ArmLifecycleImpl::SelectMode()
    {
        if (!safety.SelectMode(can::FocMotorMode::torque))
            fsm.Dispatch(Failed{ JointCommand::selectMode });
    }

    void ArmLifecycleImpl::StartJoints()
    {
        if (!safety.Start())
            fsm.Dispatch(Failed{ JointCommand::start });
    }

    void ArmLifecycleImpl::KeepStarting(lifecycle::Enabling& enabling)
    {
        if (!enabling.started)
            return;

        ++enabling.periodsStarting;
        if (enabling.periodsStarting % restartPeriods == 0)
            StartJoints();
    }

    void ArmLifecycleImpl::GiveUpStarting()
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (safety.Reading(joint).state != can::FocMotorState::running)
            {
                unstartedJoint = joint;
                break;
            }
        StopJoints();
    }

    void ArmLifecycleImpl::StopJoints()
    {
        if (!safety.Stop())
            fsm.Dispatch(Failed{ JointCommand::stop });
    }

    bool ArmLifecycleImpl::HasConfiguration() const
    {
        for (std::size_t configurationStep = 0; configurationStep != configurationSteps; ++configurationStep)
            if (Configures(configurationStep))
                return true;
        return false;
    }

    std::size_t ArmLifecycleImpl::JointOf(std::size_t configurationStep)
    {
        return configurationStep < jointCount ? configurationStep : (configurationStep - jointCount) / 2;
    }

    ConfigurationPart ArmLifecycleImpl::PartOf(std::size_t configurationStep)
    {
        if (configurationStep < jointCount)
            return ConfigurationPart::watchdog;
        return (configurationStep - jointCount) % 2 == 0 ? ConfigurationPart::motor : ConfigurationPart::controller;
    }

    bool ArmLifecycleImpl::Configures(std::size_t configurationStep) const
    {
        const auto joint = JointOf(configurationStep);
        switch (PartOf(configurationStep))
        {
            case ConfigurationPart::watchdog:
                return watchdogSeconds.has_value();
            case ConfigurationPart::motor:
                return motors[joint].has_value();
            default:
                return controllers[joint].has_value();
        }
    }

    bool ArmLifecycleImpl::StartStep(std::size_t configurationStep)
    {
        const auto joint = JointOf(configurationStep);
        switch (PartOf(configurationStep))
        {
            case ConfigurationPart::watchdog:
                return configurator.ApplyWatchdog(joint, *watchdogSeconds);
            case ConfigurationPart::motor:
                return configurator.ApplyMotor(joint, motors[joint]->settings);
            default:
                return configurator.Apply(joint, *controllers[joint]);
        }
    }

    bool ArmLifecycleImpl::WorkingOn(std::size_t joint, ConfigurationPart part) const
    {
        return fsm.Is<lifecycle::Configuring>() && configuringStep.has_value() && JointOf(*configuringStep) == joint && PartOf(*configuringStep) == part;
    }

    void ArmLifecycleImpl::StartConfiguring()
    {
        nextStep = 0;
        configuringStep.reset();
    }

    void ArmLifecycleImpl::ConfigureNext()
    {
        if (configuringStep.has_value() || configurator.Busy())
            return;

        while (nextStep != configurationSteps && !Configures(nextStep))
            ++nextStep;

        if (nextStep == configurationSteps)
        {
            fsm.Dispatch(Configured{});
            return;
        }

        configuringStep = nextStep;
        if (!StartStep(nextStep))
            Misconfigure(MisconfigurationReason::failed);
    }

    void ArmLifecycleImpl::Verified()
    {
        nextStep = *configuringStep + 1;
        configuringStep.reset();
        ConfigureNext();
    }

    void ArmLifecycleImpl::Misconfigure(MisconfigurationReason reason)
    {
        const auto step = *configuringStep;
        configuringStep.reset();
        fsm.Dispatch(lifecycle::Misconfigure{ { JointOf(step), reason, PartOf(step) } });
    }
}
