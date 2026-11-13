#include "core/joint_configurator/implementations/JointConfiguratorImpl.hpp"

namespace arm
{
    namespace
    {
        constexpr std::array<can::FocControlLoop, 3> loops{ can::FocControlLoop::current, can::FocControlLoop::speed, can::FocControlLoop::position };

        uint8_t AlgorithmOf(const ControllerSettings& settings, std::size_t loop)
        {
            switch (loop)
            {
                case 0:
                    return static_cast<uint8_t>(settings.current);
                case 1:
                    return static_cast<uint8_t>(settings.speed);
                default:
                    return static_cast<uint8_t>(settings.position);
            }
        }
    }

    JointConfiguratorImpl::JointConfiguratorImpl(JointBus& bus, const ArmLifecycle& lifecycle)
        : JointBusObserver(bus)
        , bus(bus)
        , lifecycle(lifecycle)
    {}

    bool JointConfiguratorImpl::Apply(std::size_t newJoint, const ControllerSettings& newSettings)
    {
        if (operation.has_value())
            return false;
        settings = newSettings;
        return Start(Operation::applyController, newJoint);
    }

    bool JointConfiguratorImpl::Read(std::size_t newJoint)
    {
        if (operation.has_value())
            return false;
        reading = ControllerReading{};
        return Start(Operation::readController, newJoint);
    }

    bool JointConfiguratorImpl::ApplyMotor(std::size_t newJoint, const MotorSettings& newMotor)
    {
        if (operation.has_value())
            return false;
        motor = newMotor;
        return Start(Operation::applyMotor, newJoint);
    }

    bool JointConfiguratorImpl::ReadMotor(std::size_t newJoint)
    {
        if (operation.has_value())
            return false;
        motor = MotorSettings{};
        return Start(Operation::readMotor, newJoint);
    }

    bool JointConfiguratorImpl::Busy() const
    {
        return operation.has_value();
    }

    void JointConfiguratorImpl::OnContractsResolved(can::ContractCompatibility)
    {}

    void JointConfiguratorImpl::OnJointFault(std::size_t, can::FocFaultCode)
    {}

    void JointConfiguratorImpl::OnJointSilent(std::size_t)
    {}

    void JointConfiguratorImpl::OnCommandDelivered(JointCommand)
    {}

    void JointConfiguratorImpl::OnCommandFailed(JointCommand, CommandFailure)
    {}

    void JointConfiguratorImpl::OnAlgorithms(std::size_t answering, const can::FocAlgorithms& algorithms)
    {
        if (!operation.has_value() || answering != joint || !ExpectsAlgorithms())
            return;

        if (operation == Operation::readController)
        {
            reading.mode = algorithms.mode;
            reading.settings.current = algorithms.current;
            reading.settings.speed = algorithms.speed;
            reading.settings.position = algorithms.position;
        }

        Advance();
    }

    void JointConfiguratorImpl::OnParameter(std::size_t answering, can::FocParameter parameter, can::FocParameterValue value)
    {
        const auto index = ExpectedParameter();
        if (!operation.has_value() || answering != joint || !index.has_value() || ParameterAt(*index) != parameter)
            return;

        if (operation == Operation::readController)
            reading.settings.gains[*index] = value.value;
        else if (operation == Operation::readMotor)
            motor.values[*index] = value.value;

        Advance();
    }

    bool JointConfiguratorImpl::Start(Operation newOperation, std::size_t newJoint)
    {
        const auto state = lifecycle.State();
        if (operation.has_value() || newJoint >= jointCount || (state != ArmState::ready && state != ArmState::configuring))
            return false;

        operation = newOperation;
        joint = newJoint;
        step = 0;
        SendStep();
        return true;
    }

    std::size_t JointConfiguratorImpl::StepCount() const
    {
        return AlgorithmSteps() + (OnMotor() ? motorParameterCount : gainCount);
    }

    std::size_t JointConfiguratorImpl::AlgorithmSteps() const
    {
        if (OnMotor())
            return 0;
        return operation == Operation::applyController ? loops.size() : 1;
    }

    bool JointConfiguratorImpl::Applies() const
    {
        return operation == Operation::applyController || operation == Operation::applyMotor;
    }

    bool JointConfiguratorImpl::OnMotor() const
    {
        return operation == Operation::applyMotor || operation == Operation::readMotor;
    }

    can::FocParameter JointConfiguratorImpl::ParameterAt(std::size_t index) const
    {
        return OnMotor() ? motorParameters[index] : gainParameters[index];
    }

    bool JointConfiguratorImpl::ExpectsAlgorithms() const
    {
        return step < AlgorithmSteps();
    }

    std::optional<std::size_t> JointConfiguratorImpl::ExpectedParameter() const
    {
        if (ExpectsAlgorithms())
            return std::nullopt;

        return step - AlgorithmSteps();
    }

    void JointConfiguratorImpl::SendStep()
    {
        timeoutTimer.Start(answerTimeout, [this]()
            {
                Fail(ConfigurationFailure::timedOut);
            });
        TrySend();
    }

    void JointConfiguratorImpl::TrySend()
    {
        const auto stepAtSend = step;
        if (!SendCurrentStep() && operation.has_value() && step == stepAtSend)
            retryTimer.Start(retryInterval, [this]()
                {
                    TrySend();
                });
    }

    bool JointConfiguratorImpl::SendCurrentStep()
    {
        if (ExpectsAlgorithms())
            return Applies() ? bus.SelectAlgorithm(joint, loops[step], AlgorithmOf(settings, step)) : bus.QueryAlgorithms(joint);

        const auto index = *ExpectedParameter();
        if (!Applies())
            return bus.QueryParameter(joint, ParameterAt(index));
        return bus.SetParameter(joint, ParameterAt(index), OnMotor() ? motor.values[index] : settings.gains[index]);
    }

    void JointConfiguratorImpl::Advance()
    {
        ++step;
        if (step == StepCount())
            Finish();
        else
            SendStep();
    }

    void JointConfiguratorImpl::Finish()
    {
        const auto finished = *operation;
        const auto finishedJoint = joint;
        const auto finishedReading = reading;
        const auto finishedMotor = motor;
        operation.reset();
        retryTimer.Cancel();
        timeoutTimer.Cancel();

        switch (finished)
        {
            case Operation::applyController:
                NotifyObservers([finishedJoint](auto& observer)
                    {
                        observer.OnApplied(finishedJoint);
                    });
                break;
            case Operation::readController:
                NotifyObservers([finishedJoint, &finishedReading](auto& observer)
                    {
                        observer.OnRead(finishedJoint, finishedReading);
                    });
                break;
            case Operation::applyMotor:
                NotifyObservers([finishedJoint](auto& observer)
                    {
                        observer.OnMotorApplied(finishedJoint);
                    });
                break;
            case Operation::readMotor:
                NotifyObservers([finishedJoint, &finishedMotor](auto& observer)
                    {
                        observer.OnMotorRead(finishedJoint, finishedMotor);
                    });
                break;
        }
    }

    void JointConfiguratorImpl::Fail(ConfigurationFailure failure)
    {
        const auto failedJoint = joint;
        operation.reset();
        retryTimer.Cancel();
        timeoutTimer.Cancel();
        NotifyObservers([failedJoint, failure](auto& observer)
            {
                observer.OnConfigurationFailed(failedJoint, failure);
            });
    }
}
