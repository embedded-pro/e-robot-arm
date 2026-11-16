#include "e-foc-protocol/client/FocMotorCanClient.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "tools/simulated_arm/SimulatedArm.hpp"
#include "gmock/gmock.h"

namespace
{
    using namespace std::chrono_literals;

    arm::JointLimits Wide()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -10.0f }, foc::Radians{ 10.0f } };
        return limits;
    }

    simulation::PlantConfig BentArm()
    {
        simulation::PlantConfig config{};
        config.transmissions[1].offset = foc::Radians{ 0.5f };
        config.transmissions[2].offset = foc::Radians{ -0.3f };
        return config;
    }

    arm::JointPositions BentPose()
    {
        arm::JointPositions pose{};
        pose[1] = foc::Radians{ 0.5f };
        pose[2] = foc::Radians{ -0.3f };
        return pose;
    }

    class ArmPlantTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        void Drive(std::size_t joint, float torque)
        {
            const auto node = static_cast<uint16_t>(joint + 1);
            client.CategoryClient().SendSelectControlMode(node, can::FocMotorMode::torque);
            ForwardTime(1ms);
            client.CategoryClient().SendStart(node);
            ForwardTime(1ms);
            client.CategoryClient().SendSetpoint(node, can::FocSetpoint{ can::FocSetpointKind::torqueNewtonMeter, torque });
            ForwardTime(1ms);
        }

        void DriveAll(const arm::JointTorques& torques)
        {
            for (uint16_t node = 1; node <= arm::jointCount; ++node)
                client.CategoryClient().SendSelectControlMode(node, can::FocMotorMode::torque);
            ForwardTime(1ms);
            for (uint16_t node = 1; node <= arm::jointCount; ++node)
            {
                client.CategoryClient().SendStart(node);
                client.CategoryClient().SendSetpoint(node, can::FocSetpoint{ can::FocSetpointKind::torqueNewtonMeter, torques[node - 1].Value() });
                ForwardTime(0ms);
            }
            ForwardTime(1ms);
        }

        void Observe()
        {
            for (uint16_t node = 1; node <= arm::jointCount; ++node)
                client.CategoryClient().SendRequestTelemetry(node);
            ForwardTime(1ms);
        }

        std::array<float, arm::jointCount> Positions()
        {
            Observe();
            std::array<float, arm::jointCount> positions{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                positions[joint] = arm.Joint(joint).Position().Value();
            return positions;
        }

        simulation::SimulatedArm arm{ arm::JointNodeIds{ 1, 2, 3, 4, 5, 6 }, 0ms, BentArm() };
        can::FocMotorCanClient client{ arm.ArmSide() };
        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), Wide() };
    };
}

TEST_F(ArmPlantTest, a_driven_joint_without_torque_falls_under_gravity_while_the_others_hold)
{
    const auto before = Positions();
    const auto gravity = model.GravityTorque(BentPose());
    ASSERT_GT(std::abs(gravity[1].Value()), 1.0f);

    Drive(1, 0.0f);
    ForwardTime(200ms);
    const auto after = Positions();

    EXPECT_GT(std::abs(after[1] - before[1]), 0.01f);
    EXPECT_LT((after[1] - before[1]) * gravity[1].Value(), 0.0f);
    for (const auto joint : { 0u, 2u, 3u, 4u, 5u })
        EXPECT_NEAR(before[joint], after[joint], 1.0e-6f) << "joint " << joint;
}

TEST_F(ArmPlantTest, the_gravity_torque_on_every_joint_holds_the_arm_still)
{
    DriveAll(model.GravityTorque(BentPose()));
    ForwardTime(200ms);

    for (const auto position : Positions())
        EXPECT_NEAR(0.0f, position, 1.0e-3f);
}

TEST_F(ArmPlantTest, a_joint_stopped_mid_fall_holds_where_it_stopped)
{
    Drive(1, 0.0f);
    ForwardTime(100ms);
    client.CategoryClient().SendStop(2);
    ForwardTime(1ms);
    const auto stopped = Positions();
    EXPECT_GT(std::abs(stopped[1]), 0.001f);

    ForwardTime(200ms);
    EXPECT_NEAR(stopped[1], Positions()[1], 1.0e-6f);
}
