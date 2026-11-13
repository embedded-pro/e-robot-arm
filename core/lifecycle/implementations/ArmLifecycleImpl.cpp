#include "core/lifecycle/implementations/ArmLifecycleImpl.hpp"
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
                    self.SelectMode();
                    return Enabling{};
                }),
            Machine::InternalRow<Enabling, Delivered>([](ArmLifecycleImpl& self, Enabling&, const Delivered&)
                {
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
            .Allow<Enabling, Ready, Enabled, Tripped>()
            .Allow<Enabled, Ready, Moving, Disabling, Tripped>()
            .Allow<Moving, Enabled, Ready, Tripped>()
            .Allow<Disabling, Ready, Tripped>()
            .Allow<Tripped, WaitingForCalibration>();
    }

    ArmLifecycleImpl::ArmLifecycleImpl(SafetySupervisor& safety, JointMotion& motion, JointConfigurator& configurator, can::FocMotorMode enableMode,
        const JointControllers& controllers, const JointMotors& motors)
        : JointBusObserver(safety)
        , JointMotionObserver(motion)
        , JointConfiguratorObserver(configurator)
        , safety(safety)
        , motion(motion)
        , configurator(configurator)
        , enableMode(enableMode)
        , controllers(controllers)
        , motors(motors)
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
        return fsm.Is<Tripped>() && safety.Rearm() && Accepted(Rearmed{});
    }

    bool ArmLifecycleImpl::MoveTo(const JointPositions& target)
    {
        return fsm.Is<Enabled>() && motion.MoveTo(target) && Accepted(MoveStarted{});
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

    void ArmLifecycleImpl::OnConfigurationFailed(std::size_t joint, ConfigurationFailure)
    {
        if (fsm.Is<Configuring>() && configuringStep.has_value() && *configuringStep / 2 == joint)
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
        if (!safety.SelectMode(enableMode))
            fsm.Dispatch(Failed{ JointCommand::selectMode });
    }

    void ArmLifecycleImpl::StartJoints()
    {
        if (!safety.Start())
            fsm.Dispatch(Failed{ JointCommand::start });
    }

    void ArmLifecycleImpl::StopJoints()
    {
        if (!safety.Stop())
            fsm.Dispatch(Failed{ JointCommand::stop });
    }

    bool ArmLifecycleImpl::HasConfiguration() const
    {
        for (std::size_t configurationStep = 0; configurationStep != 2 * jointCount; ++configurationStep)
            if (Configures(configurationStep))
                return true;
        return false;
    }

    bool ArmLifecycleImpl::Configures(std::size_t configurationStep) const
    {
        const auto joint = configurationStep / 2;
        return configurationStep % 2 == 0 ? motors[joint].has_value() : controllers[joint].has_value();
    }

    bool ArmLifecycleImpl::WorkingOn(std::size_t joint, ConfigurationPart part) const
    {
        return fsm.Is<lifecycle::Configuring>() && configuringStep == 2 * joint + static_cast<std::size_t>(part);
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

        while (nextStep != 2 * jointCount && !Configures(nextStep))
            ++nextStep;

        if (nextStep == 2 * jointCount)
        {
            fsm.Dispatch(Configured{});
            return;
        }

        configuringStep = nextStep;
        const auto joint = nextStep / 2;
        const bool started = nextStep % 2 == 0 ? configurator.ApplyMotor(joint, motors[joint]->settings) : configurator.Apply(joint, *controllers[joint]);
        if (!started)
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
        fsm.Dispatch(lifecycle::Misconfigure{ { step / 2, reason, step % 2 == 0 ? ConfigurationPart::motor : ConfigurationPart::controller } });
    }
}
