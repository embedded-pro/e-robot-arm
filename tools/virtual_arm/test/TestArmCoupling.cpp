#include "tools/virtual_arm/ArmCoupling.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    using namespace std::chrono_literals;

    constexpr float tolerance{ 1.0e-5f };

    class ArmCouplingTest
        : public testing::Test
    {
    public:
        ArmCouplingTest()
        {
            for (auto& limit : config.positionLimits)
                limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
            config.transmissions[1] = { 50.0f, arm::JointDirection::negative, foc::Radians{ 0.2f } };
        }

        virtual_arm::MotorStates Motors(const arm::JointPositions& joints, const arm::JointSpeeds& speeds) const
        {
            virtual_arm::MotorStates motors{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                motors[joint] = { config.transmissions[joint].MotorPosition(joints[joint]).Value(), config.transmissions[joint].MotorSpeed(speeds[joint]).Value() };
            return motors;
        }

        arm::ArmConfig config{ { 1, 2, 3, 4, 5, 6 }, 3, {}, {}, 10ms };
        arm::JointPositions joints{ foc::Radians{ 0.3f }, foc::Radians{ 0.6f }, foc::Radians{ -0.4f }, foc::Radians{ 0.1f }, foc::Radians{ 0.5f }, foc::Radians{ 0.0f } };
    };
}

TEST_F(ArmCouplingTest, at_rest_each_shaft_carries_its_links_weight_through_the_transmission)
{
    const virtual_arm::ArmCoupling coupling{ config };
    const arm::ArmModel model{ config.geometry, config.mass, config.positionLimits };
    const auto gravity = model.GravityTorque(joints);

    const auto torques = coupling.ShaftTorques(Motors(joints, {}));

    EXPECT_NEAR(-gravity[0].Value(), torques[0], tolerance);
    EXPECT_NEAR(gravity[1].Value() / 50.0f, torques[1], tolerance);
    EXPECT_GT(std::abs(gravity[1].Value()), 0.1f);
}

TEST_F(ArmCouplingTest, moving_joints_add_their_speed_dependent_coupling_and_a_collision_adds_its_torque)
{
    const virtual_arm::ArmCoupling coupling{ config };
    const arm::ArmModel model{ config.geometry, config.mass, config.positionLimits };
    arm::JointSpeeds speeds{};
    speeds.fill(foc::RadiansPerSecond{ 1.5f });
    arm::JointTorques collision{};
    collision[2] = foc::NewtonMeter{ 4.0f };

    const auto held = model.InverseDynamics(joints, speeds, arm::JointAccelerations{});
    const auto torques = coupling.ShaftTorques(Motors(joints, speeds), collision);

    EXPECT_NEAR(held[1].Value() / 50.0f, torques[1], tolerance);
    EXPECT_NEAR(4.0f - held[2].Value(), torques[2], tolerance);
    EXPECT_GT(std::abs(held[3].Value() - model.GravityTorque(joints)[3].Value()), 1.0e-4f);
}
