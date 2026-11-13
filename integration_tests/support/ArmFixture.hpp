#ifndef INTEGRATION_TESTS_SUPPORT_ARM_FIXTURE_HPP
#define INTEGRATION_TESTS_SUPPORT_ARM_FIXTURE_HPP

#include "core/arm/Arm.hpp"
#include "e-foc-protocol/client/FocMotorCanClient.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "tools/simulated_arm/SimulatedArm.hpp"
#include <optional>
#include <vector>

namespace integration
{
    class ArmEvents
        : private arm::JointBusObserver
        , private arm::JointMotionObserver
        , private arm::JointConfiguratorObserver
    {
    public:
        ArmEvents(arm::SafetySupervisor& safety, arm::JointMotion& motion, arm::JointConfigurator& configurator);

        std::optional<can::ContractCompatibility> contracts;
        std::vector<std::size_t> faultedJoints;
        std::vector<std::size_t> silentJoints;
        std::vector<arm::JointCommand> deliveredCommands;
        std::vector<arm::JointCommand> failedCommands;
        std::optional<std::size_t> applied;
        std::optional<arm::ControllerReading> read;
        bool configurationFailed{ false };
        uint32_t movesDone{ 0 };
        uint32_t movesAborted{ 0 };

    private:
        void OnContractsResolved(can::ContractCompatibility compatibility) override;
        void OnJointFault(std::size_t joint, can::FocFaultCode fault) override;
        void OnJointSilent(std::size_t joint) override;
        void OnCommandDelivered(arm::JointCommand command) override;
        void OnCommandFailed(arm::JointCommand command, arm::CommandFailure failure) override;
        void OnApplied(std::size_t joint) override;
        void OnRead(std::size_t joint, const arm::ControllerReading& reading) override;
        void OnConfigurationFailed(std::size_t joint, arm::ConfigurationFailure failure) override;
        void OnMoveDone() override;
        void OnMoveAborted() override;
    };

    struct ArmFixture
        : infra::ClockFixture
    {
        explicit ArmFixture(const arm::JointTransmissions& transmissions = {}, const arm::JointControllers& controllers = {});

        simulation::SimulatedJoint& Joint(std::size_t number);
        void StartInPositionMode();

        simulation::SimulatedArm simulatedArm;
        can::FocMotorCanClient client{ simulatedArm.ArmSide() };
        arm::Arm arm;
        ArmEvents events{ arm.Safety(), arm.Motion(), arm.Configurator() };
        arm::ControllerSettings controller{};
        arm::JointPositions start{};
        arm::JointPositions target{};
        bool lastAccepted{ false };
        uint32_t movesDoneBefore{ 0 };
    };
}

#endif
