#include "core/arm_controllers/implementations/OperationalSpace.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "gmock/gmock.h"
#include <cmath>

namespace
{
    constexpr float tolerance{ 1.0e-3f };

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

    class OperationalSpaceTest
        : public testing::Test
    {
    public:
        OperationalSpaceTest()
        {
            measured.positions = Joints(0.2f, 0.5f, 0.6f, 0.3f, 0.9f, -0.2f);
        }

        math::Vector<float, 6> ToolAcceleration(const arm::JointTorques& torques) const
        {
            const auto angles = model.ModelVector(measured.positions);
            const auto speeds = model.ModelSpeeds(measured.speeds);
            auto inertia = model.Dynamics().ComputeMassMatrix(angles);
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                inertia.at(joint, joint) += rotors[joint];
            const arm::ArmModel::JointVector free{ model.ModelTorques(torques) - model.Dynamics().ComputeCoriolisTerms(angles, speeds) -
                                                   model.Dynamics().ComputeGravityTerms(angles) };
            const auto jointAcceleration = solvers::SolveSystem(inertia, free);
            return math::Vector<float, 6>{ provider.Jacobian(angles) * jointAcceleration + provider.BiasAcceleration(angles, speeds) };
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        arm::ArmTaskJacobian provider{ model };
        const arm::JointInertias rotors{ 0.01f, 0.01f, 0.01f, 0.001f, 0.001f, 0.001f };
        arm::OperationalSpace controller{ model, rotors, arm::OperationalSpaceTuning{ 20.0f, 0.5f, 0.0f }, 0.0f };
        arm::MeasuredJoints measured{};
    };
}

TEST_F(OperationalSpaceTest, holds_the_reference_at_rest_with_the_gravity_torque_alone_and_takes_over_without_a_step)
{
    EXPECT_TRUE(controller.BumplessTransfer());
    controller.Reset(measured, arm::AtRest(measured.positions));
    const auto setpoints = controller.Step(measured, arm::AtRest(measured.positions));
    const auto gravity = model.GravityTorque(measured.positions);

    EXPECT_EQ(measured.positions, setpoints.positions);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(gravity[joint].Value(), setpoints.torques[joint].Value(), tolerance) << "joint " << joint;
}

TEST_F(OperationalSpaceTest, gives_the_tool_the_references_acceleration_plus_the_pose_and_twist_corrections)
{
    auto reference = arm::AtRest(measured.positions);
    reference.positions[1] = foc::Radians{ reference.positions[1].Value() + 0.02f };
    reference.speeds[2] = foc::RadiansPerSecond{ 0.3f };
    reference.accelerations[0] = foc::RadiansPerSecondSquared{ 1.0f };
    measured.speeds[4] = foc::RadiansPerSecond{ 0.5f };

    const auto torques = controller.Step(measured, reference).torques;

    const auto target = model.ModelVector(reference.positions);
    const auto targetSpeeds = model.ModelSpeeds(reference.speeds);
    const auto angles = model.ModelVector(measured.positions);
    const auto targetJacobian = provider.Jacobian(target);
    const math::Vector<float, 6> twist{ targetJacobian * targetSpeeds };
    const math::Vector<float, 6> feedforward{ targetJacobian * model.ModelAccelerations(reference.accelerations) + provider.BiasAcceleration(target, targetSpeeds) };
    const auto error = math::SE3Transform<float>::PoseError(provider.ToolPose(target), provider.ToolPose(angles));
    const math::Vector<float, 6> errorRate{ twist - provider.Jacobian(angles) * model.ModelSpeeds(measured.speeds) };

    const auto achieved = ToolAcceleration(torques);
    for (std::size_t axis = 0; axis != 6; ++axis)
    {
        const auto expected = feedforward.at(axis, 0) + 2.0f * 0.5f * 20.0f * errorRate.at(axis, 0) + 400.0f * error.at(axis, 0);
        EXPECT_NEAR(expected, achieved.at(axis, 0), 1.0e-2f * (1.0f + std::abs(expected))) << axis;
    }
}

TEST_F(OperationalSpaceTest, hands_over_to_joint_space_computed_torque_as_the_manipulability_falls_below_the_profiles_minimum)
{
    arm::OperationalSpace blended{ model, rotors, arm::OperationalSpaceTuning{ 20.0f, 0.5f, 0.01f }, 0.001f };
    EXPECT_FLOAT_EQ(0.0f, blended.JointSpaceShare(measured.positions));
    EXPECT_FLOAT_EQ(1.0f, blended.JointSpaceShare(arm::JointPositions{}));
    const auto between = Joints(0.08f, 0.2f, 0.24f, 0.12f, 0.36f, -0.08f);
    const auto share = blended.JointSpaceShare(between);
    EXPECT_NEAR((0.001f - model.Manipulability(between)) / 0.00075f, share, 1.0e-4f);
    EXPECT_GT(share, 0.0f);
    EXPECT_LT(share, 1.0f);

    measured.positions = arm::JointPositions{};
    auto reference = arm::AtRest(Joints(0.0f, 0.01f, 0.02f, 0.0f, 0.03f, 0.0f));
    const auto torques = blended.Step(measured, reference).torques;

    const auto angles = model.ModelVector(measured.positions);
    auto inertia = model.Dynamics().ComputeMassMatrix(angles);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        inertia.at(joint, joint) += rotors[joint];
    const arm::ArmModel::JointVector acceleration{ (model.ModelVector(reference.positions) - angles) * 400.0f };
    const auto expected = model.ToJointTorques(arm::ArmModel::JointVector{ inertia * acceleration + model.Dynamics().ComputeGravityTerms(angles) });
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(expected[joint].Value(), torques[joint].Value(), tolerance) << "joint " << joint;
}

TEST_F(OperationalSpaceTest, stays_finite_at_the_stretched_singular_home_without_the_hand_over)
{
    measured.positions = arm::JointPositions{};
    arm::OperationalSpace damped{ model, rotors, arm::OperationalSpaceTuning{}, 0.0f };
    EXPECT_FLOAT_EQ(0.0f, damped.JointSpaceShare(measured.positions));
    const auto torques = damped.Step(measured, arm::AtRest(Joints(0.0f, 0.02f, 0.02f, 0.0f, 0.02f, 0.0f))).torques;
    for (const auto& torque : torques)
        EXPECT_TRUE(std::isfinite(torque.Value()));
}
