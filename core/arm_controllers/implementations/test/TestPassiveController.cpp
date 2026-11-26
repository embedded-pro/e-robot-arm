#include "core/arm_controllers/implementations/PassiveController.hpp"
#include "gmock/gmock.h"

namespace
{
    constexpr float tolerance{ 1.0e-4f };

    arm::JointLimits Limits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -1.0f }, foc::Radians{ 1.0f } };
        return limits;
    }

    class PassiveControllerTest
        : public testing::Test
    {
    public:
        PassiveControllerTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                measured.positions[joint] = foc::Radians{ 0.1f * static_cast<float>(joint) };
        }

        float Inertia(std::size_t joint) const
        {
            return drivenArm.JointMassMatrix(measured.positions).at(joint, joint);
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), Limits() };
        const arm::JointInertias rotors{ 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f };
        arm::DrivenArm drivenArm{ model, rotors };
        arm::PassiveController free{ drivenArm, Limits(), arm::PassiveLaw::free };
        arm::PassiveController gravityFloat{ drivenArm, Limits(), arm::PassiveLaw::gravityFloat };
        arm::MeasuredJoints measured{};
    };
}

TEST_F(PassiveControllerTest, free_sends_no_torque_away_from_the_limits_and_reports_the_measured_pose)
{
    measured.speeds[2] = foc::RadiansPerSecond{ 1.5f };

    const auto setpoints = free.Step(measured, arm::AtRest(arm::JointPositions{}));

    EXPECT_EQ(measured.positions, setpoints.positions);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(0.0f, setpoints.torques[joint].Value(), tolerance) << "joint " << joint;
}

TEST_F(PassiveControllerTest, gravity_float_sends_the_gravity_torque_alone)
{
    const auto setpoints = gravityFloat.Step(measured, arm::AtRest(arm::JointPositions{}));
    const auto gravity = model.GravityTorque(measured.positions);

    EXPECT_EQ(measured.positions, setpoints.positions);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(gravity[joint].Value(), setpoints.torques[joint].Value(), tolerance) << "joint " << joint;
    EXPECT_GT(std::abs(gravity[1].Value()), 1.0f);
}

TEST_F(PassiveControllerTest, a_wall_inside_the_band_before_the_upper_limit_pushes_back_and_damps)
{
    measured.positions[3] = foc::Radians{ 0.95f };
    measured.speeds[3] = foc::RadiansPerSecond{ 0.5f };

    const auto torque = free.Step(measured, arm::AtRest(arm::JointPositions{})).torques[3].Value();

    EXPECT_NEAR(-400.0f * Inertia(3) * 0.05f - 40.0f * Inertia(3) * 0.5f, torque, tolerance);
}

TEST_F(PassiveControllerTest, a_wall_inside_the_band_before_the_lower_limit_pushes_up)
{
    measured.positions[2] = foc::Radians{ -0.97f };

    const auto torque = free.Step(measured, arm::AtRest(arm::JointPositions{})).torques[2].Value();

    EXPECT_NEAR(400.0f * Inertia(2) * 0.07f, torque, tolerance);
}

TEST_F(PassiveControllerTest, a_wall_adds_to_the_gravity_torque_under_gravity_float)
{
    measured.positions[1] = foc::Radians{ 0.92f };

    const auto torque = gravityFloat.Step(measured, arm::AtRest(arm::JointPositions{})).torques[1].Value();

    EXPECT_NEAR(model.GravityTorque(measured.positions)[1].Value() - 400.0f * Inertia(1) * 0.02f, torque, 1.0e-3f);
}

TEST_F(PassiveControllerTest, never_takes_over_a_running_move)
{
    EXPECT_FALSE(free.BumplessTransfer());
    EXPECT_FALSE(gravityFloat.BumplessTransfer());
}
