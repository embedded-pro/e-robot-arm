#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"
#include "gmock/gmock.h"

namespace
{
    constexpr float tolerance{ 1.0e-4f };

    arm::JointLimits WideLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        return limits;
    }

    arm::JointPdGains Gains()
    {
        arm::JointPdGains gains{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            gains[joint] = { 10.0f * static_cast<float>(joint + 1), 0.5f };
        return gains;
    }

    class GravityCompensatedPdTest
        : public testing::Test
    {
    public:
        GravityCompensatedPdTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                measured.positions[joint] = foc::Radians{ 0.1f * static_cast<float>(joint) };
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        arm::GravityCompensatedPd controller{ model, Gains() };
        arm::MeasuredJoints measured{};
    };
}

TEST_F(GravityCompensatedPdTest, holds_a_joint_at_rest_on_its_reference_with_the_gravity_torque_alone)
{
    controller.Reset(measured);
    const auto setpoints = controller.Step(measured, measured.positions);
    const auto gravity = model.GravityTorque(measured.positions);

    ASSERT_TRUE(setpoints.torques.has_value());
    EXPECT_EQ(measured.positions, setpoints.positions);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(gravity[joint].Value(), (*setpoints.torques)[joint].Value(), tolerance) << "joint " << joint;
    EXPECT_GT(std::abs(gravity[1].Value()), 1.0f);
}

TEST_F(GravityCompensatedPdTest, adds_a_stiffness_on_the_position_error_and_a_damping_on_the_speed)
{
    auto reference = measured.positions;
    reference[2] = foc::Radians{ reference[2].Value() + 0.05f };
    measured.speeds[4] = foc::RadiansPerSecond{ 2.0f };

    const auto torques = *controller.Step(measured, reference).torques;
    const auto gravity = model.GravityTorque(measured.positions);

    EXPECT_NEAR(gravity[2].Value() + 30.0f * 0.05f, torques[2].Value(), tolerance);
    EXPECT_NEAR(gravity[4].Value() - 0.5f * 2.0f, torques[4].Value(), tolerance);
}

TEST_F(GravityCompensatedPdTest, runs_the_joints_in_torque_mode_and_takes_over_without_a_step)
{
    EXPECT_EQ(can::FocMotorMode::torque, controller.Mode());
    EXPECT_TRUE(controller.BumplessTransfer());
}
