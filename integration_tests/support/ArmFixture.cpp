#include "integration_tests/support/ArmFixture.hpp"
#include "gtest/gtest.h"
#include <chrono>
#include <numbers>

namespace integration
{
    namespace
    {
        using namespace std::chrono_literals;

        constexpr arm::JointNodeIds nodes{ 11, 12, 13, 14, 15, 16 };

        arm::ArmConfig Config(const arm::JointTransmissions& transmissions, const arm::JointControllers& controllers)
        {
            arm::ArmConfig config{ nodes, 3, {}, {}, 10ms };
            config.transmissions = transmissions;
            config.controllers = controllers;
            for (auto& limit : config.positionLimits)
                limit = { foc::Radians{ -std::numbers::pi_v<float> }, foc::Radians{ std::numbers::pi_v<float> } };
            for (auto& limit : config.motionLimits)
                limit = { foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecondSquared{ 2.0f } };
            return config;
        }

    }

    ArmEvents::ArmEvents(arm::SafetySupervisor& safety, arm::JointMotion& motion, arm::JointConfigurator& configurator)
        : arm::JointBusObserver(safety)
        , arm::JointMotionObserver(motion)
        , arm::JointConfiguratorObserver(configurator)
    {}

    void ArmEvents::OnApplied(std::size_t joint)
    {
        applied = joint;
    }

    void ArmEvents::OnRead(std::size_t, const arm::ControllerReading& reading)
    {
        read = reading;
    }

    void ArmEvents::OnConfigurationFailed(std::size_t, arm::ConfigurationFailure)
    {
        configurationFailed = true;
    }

    void ArmEvents::OnContractsResolved(can::ContractCompatibility compatibility)
    {
        contracts = compatibility;
    }

    void ArmEvents::OnJointFault(std::size_t joint, can::FocFaultCode)
    {
        faultedJoints.push_back(joint);
    }

    void ArmEvents::OnJointSilent(std::size_t joint)
    {
        silentJoints.push_back(joint);
    }

    void ArmEvents::OnCommandDelivered(arm::JointCommand command)
    {
        deliveredCommands.push_back(command);
    }

    void ArmEvents::OnCommandFailed(arm::JointCommand command, arm::CommandFailure)
    {
        failedCommands.push_back(command);
    }

    void ArmEvents::OnMoveDone()
    {
        ++movesDone;
    }

    void ArmEvents::OnMoveAborted()
    {
        ++movesAborted;
    }

    ArmFixture::ArmFixture(const arm::JointTransmissions& transmissions, const arm::JointControllers& controllers)
        : simulatedArm(nodes, infra::Duration{})
        , arm(client.CategoryClient(), Config(transmissions, controllers))
    {}

    simulation::SimulatedJoint& ArmFixture::Joint(std::size_t number)
    {
        return simulatedArm.Joint(number - 1);
    }

    void ArmFixture::StartInPositionMode()
    {
        EXPECT_TRUE(arm.Safety().QueryContracts());
        ForwardTime(20ms);
        EXPECT_TRUE(arm.Safety().SelectMode(can::FocMotorMode::position));
        EXPECT_TRUE(arm.Safety().Start());
        ForwardTime(20ms);
        EXPECT_TRUE(events.failedCommands.empty());

        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            start[joint] = arm.Safety().Reading(joint).position;
    }
}
