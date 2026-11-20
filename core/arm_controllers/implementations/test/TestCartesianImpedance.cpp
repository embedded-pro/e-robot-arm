#include "core/arm_controllers/implementations/CartesianImpedance.hpp"
#include "core/arm_controllers/implementations/JointImpedance.hpp"
#include "gmock/gmock.h"
#include <cmath>

namespace
{
    constexpr float tolerance{ 1.0e-3f };
    constexpr std::array<float, 6> stiffness{ 50.0f, 50.0f, 50.0f, 0.07f, 0.07f, 0.07f };

    arm::JointLimits WideLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        return limits;
    }

    arm::JointPositions Joints(float q1, float q2, float q3, float q4, float q5, float q6)
    {
        return { foc::Radians{ q1 }, foc::Radians{ q2 }, foc::Radians{ q3 }, foc::Radians{ q4 }, foc::Radians{ q5 }, foc::Radians{ q6 } };
    }

    class CartesianImpedanceTest
        : public testing::Test
    {
    public:
        CartesianImpedanceTest()
        {
            measured.positions = Joints(0.2f, 0.5f, 0.6f, 0.3f, 0.9f, -0.2f);
        }

        std::array<float, arm::jointCount> SpringTorques(const arm::JointReference& reference) const
        {
            const auto error = math::SE3Transform<float>::PoseError(model.ToolPose(reference.positions), model.ToolPose(measured.positions));
            const auto jacobian = provider.Jacobian(model.ModelVector(measured.positions));
            std::array<float, arm::jointCount> torques{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                for (std::size_t axis = 0; axis != 6; ++axis)
                    torques[joint] += jacobian.at(axis, joint) * stiffness[axis] * error[axis];
            return torques;
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        arm::ArmTaskJacobian provider{ model };
        const arm::JointInertias rotors{ 0.01f, 0.01f, 0.01f, 0.001f, 0.001f, 0.001f };
        arm::CartesianImpedance controller{ model, rotors, arm::CartesianImpedanceTuning{} };
        arm::MeasuredJoints measured{};
    };
}

TEST_F(CartesianImpedanceTest, holds_the_reference_at_rest_with_the_gravity_torque_alone_and_takes_over_without_a_step)
{
    EXPECT_TRUE(controller.BumplessTransfer());
    controller.Reset(measured, arm::AtRest(measured.positions));
    const auto setpoints = controller.Step(measured, arm::AtRest(measured.positions));
    const auto gravity = model.GravityTorque(measured.positions);

    EXPECT_EQ(measured.positions, setpoints.positions);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(gravity[joint].Value(), setpoints.torques[joint].Value(), tolerance) << "joint " << joint;
}

TEST_F(CartesianImpedanceTest, feeds_the_references_inverse_dynamics_forward_and_pulls_the_tool_through_the_jacobian_transpose)
{
    auto reference = arm::AtRest(measured.positions);
    reference.positions[1] = foc::Radians{ reference.positions[1].Value() + 0.05f };
    reference.positions[4] = foc::Radians{ reference.positions[4].Value() - 0.04f };
    reference.accelerations[2] = foc::RadiansPerSecondSquared{ 1.5f };

    const auto torques = controller.Step(measured, reference).torques;
    const auto forward = model.InverseDynamics(measured.positions, reference.speeds, reference.accelerations);
    const auto spring = SpringTorques(reference);

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(forward[joint].Value() + spring[joint], torques[joint].Value(), tolerance) << "joint " << joint;
}

TEST_F(CartesianImpedanceTest, damps_with_the_coupled_inertia_at_the_measured_pose_and_the_stiffness_the_tool_spring_puts_on_each_joint)
{
    const auto angles = model.ModelVector(measured.positions);
    auto inertia = model.Dynamics().ComputeMassMatrix(angles);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        inertia.at(joint, joint) += rotors[joint];
    const auto jacobian = provider.Jacobian(angles);
    arm::JointStiffnesses jointStiffness{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        for (std::size_t axis = 0; axis != 6; ++axis)
            jointStiffness[joint] += stiffness[axis] * jacobian.at(axis, joint) * jacobian.at(axis, joint);

    const auto expected = arm::CoupledDamping(inertia, jointStiffness, 0.7f);
    const auto damping = controller.Damping(angles);
    for (std::size_t row = 0; row != arm::jointCount; ++row)
        for (std::size_t column = 0; column != arm::jointCount; ++column)
            EXPECT_NEAR(expected.at(row, column), damping.at(row, column), 1.0e-6f) << row << ", " << column;
    EXPECT_NEAR(2.0f * 0.7f * std::sqrt(jointStiffness[2] * inertia.at(2, 2)), damping.at(2, 2), 1.0e-5f);

    measured.speeds[2] = foc::RadiansPerSecond{ 1.0f };
    measured.speeds[5] = foc::RadiansPerSecond{ -2.0f };
    const auto torques = controller.Step(measured, arm::AtRest(measured.positions)).torques;
    const auto forward = model.InverseDynamics(measured.positions, arm::JointSpeeds{}, arm::JointAccelerations{});
    float power{ 0.0f };
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        power += (torques[joint].Value() - forward[joint].Value()) * measured.speeds[joint].Value();
    EXPECT_LT(power, 0.0f);
}

TEST_F(CartesianImpedanceTest, stays_finite_at_the_stretched_singular_home)
{
    measured.positions = arm::JointPositions{};
    measured.speeds[1] = foc::RadiansPerSecond{ 0.5f };
    const auto damping = controller.Damping(model.ModelVector(measured.positions));
    for (std::size_t row = 0; row != arm::jointCount; ++row)
        for (std::size_t column = 0; column != arm::jointCount; ++column)
            EXPECT_TRUE(std::isfinite(damping.at(row, column)));

    const auto torques = controller.Step(measured, arm::AtRest(Joints(0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f))).torques;
    for (const auto& torque : torques)
        EXPECT_TRUE(std::isfinite(torque.Value()));
}

TEST_F(CartesianImpedanceTest, softens_to_a_share_of_the_profiles_stiffness_and_refuses_one_outside_a_tenth_to_all)
{
    auto reference = arm::AtRest(measured.positions);
    reference.positions[1] = foc::Radians{ reference.positions[1].Value() + 0.05f };
    const auto gravity = model.GravityTorque(measured.positions);
    const auto full = controller.Step(measured, reference).torques;

    EXPECT_FLOAT_EQ(1.0f, controller.StiffnessShare());
    EXPECT_TRUE(controller.SetStiffnessShare(0.5f));
    EXPECT_FLOAT_EQ(0.5f, controller.StiffnessShare());
    const auto half = controller.Step(measured, reference).torques;
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(0.5f * (full[joint].Value() - gravity[joint].Value()), half[joint].Value() - gravity[joint].Value(), tolerance) << "joint " << joint;

    EXPECT_FALSE(controller.SetStiffnessShare(0.05f));
    EXPECT_FALSE(controller.SetStiffnessShare(1.5f));
    EXPECT_FALSE(controller.SetStiffnessShare(std::nanf("")));
    EXPECT_FLOAT_EQ(0.5f, controller.StiffnessShare());
}
