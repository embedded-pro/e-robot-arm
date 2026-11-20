#include "core/arm_controllers/implementations/JointImpedance.hpp"
#include "gmock/gmock.h"
#include <cmath>

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

    arm::JointImpedanceTuning Tuning()
    {
        return { arm::JointStiffnesses{ 40.0f, 80.0f, 60.0f, 10.0f, 8.0f, 4.0f }, 0.5f };
    }

    class JointImpedanceTest
        : public testing::Test
    {
    public:
        JointImpedanceTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                measured.positions[joint] = foc::Radians{ 0.1f * static_cast<float>(joint) };
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        const arm::JointInertias inertias{ arm::HomeInertias(model, arm::JointInertias{ 0.01f, 0.01f, 0.01f, 0.001f, 0.001f, 0.001f }) };
        arm::JointImpedance controller{ model, inertias, Tuning() };
        arm::MeasuredJoints measured{};
    };
}

TEST_F(JointImpedanceTest, holds_the_reference_at_rest_with_the_gravity_torque_alone_and_takes_over_without_a_step)
{
    EXPECT_TRUE(controller.BumplessTransfer());
    controller.Reset(measured, arm::AtRest(measured.positions));
    const auto setpoints = controller.Step(measured, arm::AtRest(measured.positions));
    const auto gravity = model.GravityTorque(measured.positions);

    EXPECT_EQ(measured.positions, setpoints.positions);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(gravity[joint].Value(), setpoints.torques[joint].Value(), tolerance) << "joint " << joint;
}

TEST_F(JointImpedanceTest, pulls_each_joint_with_its_stiffness_and_damps_its_speed_error)
{
    auto reference = arm::AtRest(measured.positions);
    reference.positions[1] = foc::Radians{ reference.positions[1].Value() + 0.05f };
    reference.speeds[4] = foc::RadiansPerSecond{ 1.0f };
    measured.speeds[4] = foc::RadiansPerSecond{ 3.0f };

    const auto torques = controller.Step(measured, reference).torques;
    const auto gravity = model.GravityTorque(measured.positions);

    EXPECT_NEAR(gravity[1].Value() + 80.0f * 0.05f, torques[1].Value(), tolerance);
    const auto damping = 2.0f * 0.5f * std::sqrt(8.0f * inertias[4]);
    EXPECT_NEAR(gravity[4].Value() - damping * 2.0f, torques[4].Value(), tolerance);
    EXPECT_NEAR(gravity[0].Value(), torques[0].Value(), tolerance);
}

TEST_F(JointImpedanceTest, softens_to_a_share_of_the_profiles_stiffness_and_refuses_one_outside_a_tenth_to_all)
{
    EXPECT_FLOAT_EQ(1.0f, controller.StiffnessShare());
    EXPECT_TRUE(controller.SetStiffnessShare(0.25f));
    EXPECT_FLOAT_EQ(0.25f, controller.StiffnessShare());

    auto reference = arm::AtRest(measured.positions);
    reference.positions[1] = foc::Radians{ reference.positions[1].Value() + 0.05f };
    measured.speeds[4] = foc::RadiansPerSecond{ 2.0f };
    const auto torques = controller.Step(measured, reference).torques;
    const auto gravity = model.GravityTorque(measured.positions);
    EXPECT_NEAR(gravity[1].Value() + 0.25f * 80.0f * 0.05f, torques[1].Value(), tolerance);
    EXPECT_NEAR(gravity[4].Value() - 2.0f * 0.5f * std::sqrt(0.25f * 8.0f * inertias[4]) * 2.0f, torques[4].Value(), tolerance);

    EXPECT_FALSE(controller.SetStiffnessShare(0.05f));
    EXPECT_FALSE(controller.SetStiffnessShare(1.5f));
    EXPECT_FALSE(controller.SetStiffnessShare(std::nanf("")));
    EXPECT_FLOAT_EQ(0.25f, controller.StiffnessShare());
}

TEST_F(JointImpedanceTest, without_stiffnesses_each_joint_gets_a_ten_radian_per_second_bandwidth_on_its_inertia)
{
    const auto stiffness = arm::ImpedanceStiffness(inertias, arm::JointImpedanceTuning{});
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(100.0f * inertias[joint], stiffness[joint], 1.0e-5f) << "joint " << joint;
    EXPECT_GT(inertias[1], inertias[5]);
}
