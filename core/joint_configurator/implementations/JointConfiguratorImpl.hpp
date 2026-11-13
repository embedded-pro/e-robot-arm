#ifndef CORE_JOINT_CONFIGURATOR_IMPLEMENTATIONS_JOINT_CONFIGURATOR_IMPL_HPP
#define CORE_JOINT_CONFIGURATOR_IMPLEMENTATIONS_JOINT_CONFIGURATOR_IMPL_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/joint_configurator/interfaces/JointConfigurator.hpp"
#include "core/lifecycle/interfaces/ArmLifecycle.hpp"
#include "infra/timer/Timer.hpp"
#include <optional>

namespace arm
{
    class JointConfiguratorImpl
        : public JointConfigurator
        , private JointBusObserver
    {
    public:
        static constexpr infra::Duration retryInterval{ std::chrono::milliseconds{ 1 } };
        static constexpr infra::Duration answerTimeout{ std::chrono::milliseconds{ 100 } };

        JointConfiguratorImpl(JointBus& bus, const ArmLifecycle& lifecycle);

        bool Apply(std::size_t joint, const ControllerSettings& settings) override;
        bool Read(std::size_t joint) override;
        bool ApplyMotor(std::size_t joint, const MotorSettings& motor) override;
        bool ReadMotor(std::size_t joint) override;
        bool Busy() const override;

    private:
        enum class Operation : uint8_t
        {
            applyController,
            readController,
            applyMotor,
            readMotor
        };

        void OnContractsResolved(can::ContractCompatibility compatibility) override;
        void OnJointFault(std::size_t joint, can::FocFaultCode fault) override;
        void OnJointSilent(std::size_t joint) override;
        void OnCommandDelivered(JointCommand command) override;
        void OnCommandFailed(JointCommand command, CommandFailure failure) override;
        void OnAlgorithms(std::size_t joint, const can::FocAlgorithms& algorithms) override;
        void OnParameter(std::size_t joint, can::FocParameter parameter, can::FocParameterValue value) override;

        bool Start(Operation newOperation, std::size_t newJoint);
        std::size_t StepCount() const;
        std::size_t AlgorithmSteps() const;
        bool ExpectsAlgorithms() const;
        std::optional<std::size_t> ExpectedParameter() const;
        can::FocParameter ParameterAt(std::size_t index) const;
        bool Applies() const;
        bool OnMotor() const;
        void SendStep();
        void TrySend();
        bool SendCurrentStep();
        void Advance();
        void Finish();
        void Fail(ConfigurationFailure failure);

        JointBus& bus;
        const ArmLifecycle& lifecycle;
        std::optional<Operation> operation;
        std::size_t joint{ 0 };
        std::size_t step{ 0 };
        ControllerSettings settings{};
        ControllerReading reading{};
        MotorSettings motor{};
        infra::TimerSingleShot retryTimer;
        infra::TimerSingleShot timeoutTimer;
    };
}

#endif
