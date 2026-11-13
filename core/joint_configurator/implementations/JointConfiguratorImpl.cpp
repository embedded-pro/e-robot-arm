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
        settings = newSettings;
        return Start(Operation::apply, newJoint);
    }

    bool JointConfiguratorImpl::Read(std::size_t newJoint)
    {
        reading = ControllerReading{};
        return Start(Operation::read, newJoint);
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

        if (operation == Operation::read)
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
        const auto gain = ExpectedGain();
        if (!operation.has_value() || answering != joint || !gain.has_value() || gainParameters[*gain] != parameter)
            return;

        if (operation == Operation::read)
            reading.settings.gains[*gain] = value.value;

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
        return AlgorithmSteps() + gainCount;
    }

    std::size_t JointConfiguratorImpl::AlgorithmSteps() const
    {
        return operation == Operation::apply ? loops.size() : 1;
    }

    bool JointConfiguratorImpl::ExpectsAlgorithms() const
    {
        return step < AlgorithmSteps();
    }

    std::optional<std::size_t> JointConfiguratorImpl::ExpectedGain() const
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
        if (operation == Operation::apply && ExpectsAlgorithms())
            return bus.SelectAlgorithm(joint, loops[step], AlgorithmOf(settings, step));
        if (operation == Operation::apply)
            return bus.SetParameter(joint, gainParameters[*ExpectedGain()], settings.gains[*ExpectedGain()]);
        if (ExpectsAlgorithms())
            return bus.QueryAlgorithms(joint);
        return bus.QueryParameter(joint, gainParameters[*ExpectedGain()]);
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
        operation.reset();
        retryTimer.Cancel();
        timeoutTimer.Cancel();

        if (finished == Operation::apply)
            NotifyObservers([finishedJoint](auto& observer)
                {
                    observer.OnApplied(finishedJoint);
                });
        else
            NotifyObservers([finishedJoint, &finishedReading](auto& observer)
                {
                    observer.OnRead(finishedJoint, finishedReading);
                });
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
