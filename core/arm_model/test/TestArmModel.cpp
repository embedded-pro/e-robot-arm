#include "core/arm_model/ArmModel.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <numbers>

namespace
{
    constexpr float pi{ std::numbers::pi_v<float> };

    arm::JointLimits FullTurn()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -pi }, foc::Radians{ pi } };
        return limits;
    }

    arm::JointPositions Joints(float q1, float q2, float q3, float q4, float q5, float q6)
    {
        return { foc::Radians{ q1 }, foc::Radians{ q2 }, foc::Radians{ q3 }, foc::Radians{ q4 }, foc::Radians{ q5 }, foc::Radians{ q6 } };
    }

    arm::ArmGeometry SignedGeometry()
    {
        auto geometry = arm::ReferenceGeometry();
        geometry.opw.a2 = 0.03f;
        geometry.opw.b = 0.04f;
        geometry.opw.signs = { 1, -1, 1, -1, 1, -1 };
        geometry.opw.offsets = { 0.1f, -0.2f, 0.3f, 0.0f, 0.2f, -0.1f };
        geometry.tool = { 0.01f, 0.0f, 0.05f, 0.0f, 0.0f, 0.0f };
        geometry.base = { 0.0f, 0.0f, 0.0f, 0.2f, 0.3f, 0.0f };
        return geometry;
    }

    arm::ArmMass PayloadOnly(float mass)
    {
        arm::ArmMass arm{};
        arm.payload = { mass, { 0.01f, 0.0f, 0.05f }, {} };
        return arm;
    }

    std::array<float, arm::jointCount> Values(const arm::JointTorques& torques)
    {
        std::array<float, arm::jointCount> values{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            values[joint] = torques[joint].Value();
        return values;
    }

    class ArmModelTest
        : public testing::Test
    {
    public:
        void ExpectPosition(const arm::ArmModel::Pose& pose, float x, float y, float z, float tolerance = 1.0e-5f) const
        {
            EXPECT_NEAR(x, pose.translation[0], tolerance);
            EXPECT_NEAR(y, pose.translation[1], tolerance);
            EXPECT_NEAR(z, pose.translation[2], tolerance);
        }

        void ExpectSamePose(const arm::ArmModel::Pose& expected, const arm::ArmModel::Pose& actual) const
        {
            ExpectPosition(actual, expected.translation[0], expected.translation[1], expected.translation[2], 1.0e-4f);
            for (std::size_t row = 0; row != 3; ++row)
                for (std::size_t column = 0; column != 3; ++column)
                    EXPECT_NEAR(expected.rotation.at(row, column), actual.rotation.at(row, column), 1.0e-3f);
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), FullTurn() };
    };
}

TEST_F(ArmModelTest, places_the_tool_at_known_poses)
{
    ExpectPosition(model.ToolPose(Joints(0, 0, 0, 0, 0, 0)), 0.02f, 0.0f, 0.71f);
    ExpectPosition(model.ToolPose(Joints(0, pi / 2, 0, 0, 0, 0)), 0.58f, 0.0f, 0.15f);
    ExpectPosition(model.ToolPose(Joints(pi / 2, 0, 0, 0, 0, 0)), 0.0f, 0.02f, 0.71f);

    const auto forward = model.ToolPose(Joints(0, pi / 2, 0, 0, 0, 0));
    EXPECT_NEAR(1.0f, forward.rotation.at(0, 2), 1.0e-5f);
}

TEST_F(ArmModelTest, applies_the_tool_and_the_base_placements)
{
    auto geometry = arm::ReferenceGeometry();
    geometry.tool = { 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f };
    geometry.base = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, pi / 2 };
    const arm::ArmModel placed{ geometry, arm::ReferenceMass(), FullTurn() };

    ExpectPosition(placed.ToolPose(Joints(0, pi / 2, 0, 0, 0, 0)), 1.0f, 0.68f, 0.15f);

    const arm::Placement placement{ 0.1f, -0.2f, 0.3f, 0.4f, -0.5f, 0.6f };
    const auto back = arm::ToPlacement(arm::ToTransform(placement));
    EXPECT_NEAR(placement.roll, back.roll, 1.0e-5f);
    EXPECT_NEAR(placement.pitch, back.pitch, 1.0e-5f);
    EXPECT_NEAR(placement.yaw, back.yaw, 1.0e-5f);
    EXPECT_NEAR(placement.y, back.y, 1.0e-6f);
}

TEST_F(ArmModelTest, inverse_kinematics_returns_the_joints_a_pose_came_from_over_a_grid)
{
    for (const auto q1 : { -2.0f, -0.5f, 1.2f })
        for (const auto q2 : { -0.8f, 0.3f })
            for (const auto q3 : { -1.2f, 0.6f, 1.5f })
                for (const auto q5 : { -1.0f, 0.7f })
                {
                    const auto joints = Joints(q1, q2, q3, 0.4f, q5, -0.9f);
                    const auto target = model.ToolPose(joints);
                    const auto result = model.Inverse(target, joints);

                    ASSERT_TRUE(result.joints.has_value()) << q1 << " " << q2 << " " << q3 << " " << q5;
                    ExpectSamePose(target, model.ToolPose(*result.joints));
                    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                        EXPECT_NEAR(joints[joint].Value(), (*result.joints)[joint].Value(), 1.0e-3f) << joint;
                }
}

TEST_F(ArmModelTest, chooses_the_solution_closest_to_the_current_joints)
{
    const auto joints = Joints(0.3f, 0.9f, 0.6f, 0.2f, 0.5f, 0.1f);
    const auto target = model.ToolPose(joints);
    const auto flipped = model.Inverse(target, Joints(0.3f, 0.9f, 0.6f, 0.2f - pi, -0.5f, 0.1f - pi));

    ASSERT_TRUE(flipped.joints.has_value());
    ExpectSamePose(target, model.ToolPose(*flipped.joints));
    EXPECT_NEAR(-0.5f, (*flipped.joints)[4].Value(), 1.0e-3f);
    EXPECT_NEAR(0.2f - pi, (*flipped.joints)[3].Value(), 1.0e-3f);
}

TEST_F(ArmModelTest, refuses_a_target_out_of_reach_or_outside_the_joint_limits)
{
    auto target = model.ToolPose(Joints(0, 0.5f, 0.5f, 0, 0.5f, 0));
    const auto reachable = target;
    target.translation[0] = 2.0f;
    const auto far = model.Inverse(target, Joints(0, 0, 0, 0, 0, 0));
    EXPECT_FALSE(far.joints.has_value());
    EXPECT_EQ(arm::IkFailure::unreachable, far.failure);

    auto limits = FullTurn();
    limits[0] = { foc::Radians{ 1.0f }, foc::Radians{ 1.5f } };
    const arm::ArmModel narrow{ arm::ReferenceGeometry(), arm::ReferenceMass(), limits };
    const auto outside = narrow.Inverse(reachable, Joints(0, 0, 0, 0, 0, 0));
    EXPECT_FALSE(outside.joints.has_value());
    EXPECT_EQ(arm::IkFailure::outsideLimits, outside.failure);
}

TEST_F(ArmModelTest, the_jacobian_matches_finite_differences)
{
    const auto joints = Joints(0.4f, -0.3f, 0.8f, 0.5f, -0.7f, 0.2f);
    const auto jacobian = model.ToolJacobian(joints);
    constexpr float step{ 1.0e-3f };

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        auto plus = joints;
        auto minus = joints;
        plus[joint] = foc::Radians{ joints[joint].Value() + step };
        minus[joint] = foc::Radians{ joints[joint].Value() - step };
        const auto after = model.ToolPose(plus);
        const auto before = model.ToolPose(minus);
        const auto rotation = math::SE3Transform<float>::LogRotation(after.rotation * before.rotation.Transpose());

        for (std::size_t axis = 0; axis != 3; ++axis)
        {
            EXPECT_NEAR((after.translation[axis] - before.translation[axis]) / (2.0f * step), jacobian.at(axis, joint), 2.0e-3f) << joint << " " << axis;
            EXPECT_NEAR(rotation[axis] / (2.0f * step), jacobian.at(3 + axis, joint), 2.0e-3f) << joint << " " << axis;
        }
    }
}

TEST_F(ArmModelTest, flags_each_singularity_and_its_manipulability_falls_to_zero)
{
    EXPECT_EQ((arm::Singularities{ true, true, false }), model.Near(Joints(0, 0, 0, 0, 0, 0)));
    EXPECT_NEAR(0.0f, model.Manipulability(Joints(0, 0, 0, 0, 0, 0)), 1.0e-6f);

    const auto clear = Joints(0.1f, 0.5f, 0.6f, 0.2f, 1.0f, 0.0f);
    EXPECT_FALSE(model.Near(clear).Any());
    EXPECT_GT(model.Manipulability(clear), 1.0e-4f);

    const auto q2 = 0.3f;
    const auto q3 = std::asin((-0.02f - 0.25f * std::sin(q2)) / 0.25f) - q2;
    const auto overTheBase = model.Near(Joints(0, q2, q3, 0, 1.0f, 0));
    EXPECT_TRUE(overTheBase.shoulder);
    EXPECT_FALSE(overTheBase.wrist);
}

TEST_F(ArmModelTest, the_last_link_frame_with_the_flange_and_the_tool_is_the_tool_pose)
{
    const auto geometry = SignedGeometry();
    const arm::ArmModel signedArm{ geometry, arm::ReferenceMass(), FullTurn() };
    const auto joints = Joints(0.4f, -0.3f, 0.8f, 0.5f, -0.7f, 0.2f);
    const auto frames = signedArm.LinkPoses(joints);
    const arm::ArmModel::Pose flange{ math::Matrix3<float>::Identity(), math::Vector3<float>{ 0.0f, 0.0f, geometry.opw.c4 } };

    ExpectSamePose(signedArm.ToolPose(joints), frames[5] * flange * arm::ToTransform(geometry.tool));
    ExpectPosition(model.LinkPoses(Joints(0, 0, 0, 0, 0, 0))[2], 0.02f, 0.0f, 0.4f);
}

TEST_F(ArmModelTest, holds_the_documented_torque_of_the_reference_arm)
{
    const auto upright = model.GravityTorque(Joints(0, 0, 0, 0, 0, 0));
    EXPECT_NEAR(0.0f, upright[1].Value(), 1.0e-5f);
    EXPECT_NEAR(0.0f, upright[2].Value(), 1.0e-5f);

    const auto forward = model.GravityTorque(Joints(0, pi / 2, 0, 0, 0, 0));
    EXPECT_NEAR(0.0f, forward[0].Value(), 1.0e-5f);
    EXPECT_NEAR(-9.81f * (1.0f * 0.11f + 0.8f * 0.35f + 0.3f * 0.47f + 0.25f * 0.52f + 0.1f * 0.55f), forward[1].Value(), 1.0e-4f);
}

TEST_F(ArmModelTest, a_payload_weighs_and_moves_as_a_point_mass_at_the_tool_centre_point)
{
    constexpr float mass{ 2.0f };
    const arm::ArmModel payload{ SignedGeometry(), PayloadOnly(mass), FullTurn() };
    const auto joints = Joints(0.4f, -0.3f, 0.8f, 0.5f, -0.7f, 0.2f);
    const auto jacobian = payload.ToolJacobian(joints);
    const auto gravity = Values(payload.GravityTorque(joints));
    const auto inertia = payload.JointMassMatrix(joints);

    for (std::size_t row = 0; row != arm::jointCount; ++row)
    {
        EXPECT_NEAR(mass * 9.81f * jacobian.at(2, row), gravity[row], 1.0e-4f) << row;
        for (std::size_t column = 0; column != arm::jointCount; ++column)
        {
            float expected{ 0.0f };
            for (std::size_t axis = 0; axis != 3; ++axis)
                expected += mass * jacobian.at(axis, row) * jacobian.at(axis, column);
            EXPECT_NEAR(expected, inertia.at(row, column), 1.0e-5f) << row << " " << column;
        }
    }
}

TEST_F(ArmModelTest, the_mass_matrix_is_symmetric_positive_definite_and_drives_the_inverse_dynamics)
{
    const arm::ArmModel signedArm{ SignedGeometry(), arm::ReferenceMass(), FullTurn() };
    const auto joints = Joints(0.4f, -0.3f, 0.8f, 0.5f, -0.7f, 0.2f);
    const std::array<float, arm::jointCount> acceleration{ 1.0f, -2.0f, 0.5f, 3.0f, -1.5f, 2.5f };
    arm::JointAccelerations accelerations{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        accelerations[joint] = foc::RadiansPerSecondSquared{ acceleration[joint] };

    const auto inertia = signedArm.JointMassMatrix(joints);
    const auto gravity = Values(signedArm.GravityTorque(joints));
    const auto torques = Values(signedArm.InverseDynamics(joints, arm::JointSpeeds{}, accelerations));

    for (std::size_t row = 0; row != arm::jointCount; ++row)
    {
        float expected{ gravity[row] };
        for (std::size_t column = 0; column != arm::jointCount; ++column)
        {
            EXPECT_NEAR(inertia.at(row, column), inertia.at(column, row), 1.0e-6f);
            expected += inertia.at(row, column) * acceleration[column];
        }
        EXPECT_NEAR(expected, torques[row], 1.0e-4f) << row;
    }

    for (std::size_t row = 0; row != arm::jointCount; ++row)
        EXPECT_GT(inertia.at(row, row), 0.0f);
    float energy{ 0.0f };
    for (std::size_t row = 0; row != arm::jointCount; ++row)
        for (std::size_t column = 0; column != arm::jointCount; ++column)
            energy += acceleration[row] * inertia.at(row, column) * acceleration[column];
    EXPECT_GT(energy, 0.0f);
}

TEST_F(ArmModelTest, the_velocity_torques_deliver_half_the_rate_of_change_of_the_mass_matrix)
{
    const arm::ArmModel signedArm{ SignedGeometry(), arm::ReferenceMass(), FullTurn() };
    const std::array<float, arm::jointCount> position{ 0.4f, -0.3f, 0.8f, 0.5f, -0.7f, 0.2f };
    const std::array<float, arm::jointCount> speed{ 0.8f, -1.2f, 1.5f, -0.6f, 1.1f, 0.9f };
    constexpr float step{ 1.0e-3f };

    arm::JointPositions joints{};
    arm::JointPositions ahead{};
    arm::JointPositions behind{};
    arm::JointSpeeds speeds{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        joints[joint] = foc::Radians{ position[joint] };
        ahead[joint] = foc::Radians{ position[joint] + step * speed[joint] };
        behind[joint] = foc::Radians{ position[joint] - step * speed[joint] };
        speeds[joint] = foc::RadiansPerSecond{ speed[joint] };
    }

    const auto gravity = Values(signedArm.GravityTorque(joints));
    const auto torques = Values(signedArm.InverseDynamics(joints, speeds, arm::JointAccelerations{}));
    const auto after = signedArm.JointMassMatrix(ahead);
    const auto before = signedArm.JointMassMatrix(behind);

    float power{ 0.0f };
    float change{ 0.0f };
    for (std::size_t row = 0; row != arm::jointCount; ++row)
    {
        power += speed[row] * (torques[row] - gravity[row]);
        for (std::size_t column = 0; column != arm::jointCount; ++column)
            change += speed[row] * (after.at(row, column) - before.at(row, column)) / (2.0f * step) * speed[column];
    }

    EXPECT_NEAR(0.5f * change, power, 2.0e-3f);
}

TEST_F(ArmModelTest, each_call_costs_well_under_a_fifth_of_the_control_period)
{
    constexpr int calls{ 20000 };
    const auto measure = [](auto&& call)
    {
        const auto start = std::chrono::steady_clock::now();
        for (int index = 0; index != calls; ++index)
            call(static_cast<float>(index % 100) * 0.01f);
        return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count() / calls;
    };

    volatile float sink{ 0.0f };
    const auto pose = measure([this, &sink](float q)
        {
            sink = model.ToolPose(Joints(q, 0.3f, 0.5f, q, 0.7f, q)).translation[0];
        });
    const auto jacobian = measure([this, &sink](float q)
        {
            sink = model.ToolJacobian(Joints(q, 0.3f, 0.5f, q, 0.7f, q)).at(0, 0);
        });
    const auto near = measure([this, &sink](float q)
        {
            sink = model.Near(Joints(q, 0.3f, 0.5f, q, 0.7f, q)).Any() ? 1.0f : 0.0f;
        });
    const auto target = model.ToolPose(Joints(0.2f, 0.3f, 0.5f, 0.1f, 0.7f, 0.4f));
    const auto inverse = measure([this, &sink, &target](float q)
        {
            sink = model.Inverse(target, Joints(q, 0.3f, 0.5f, q, 0.7f, q)).joints.has_value() ? 1.0f : 0.0f;
        });

    const auto gravity = measure([this, &sink](float q)
        {
            sink = model.GravityTorque(Joints(q, 0.3f, 0.5f, q, 0.7f, q))[1].Value();
        });
    arm::JointSpeeds speeds{};
    arm::JointAccelerations accelerations{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        speeds[joint] = foc::RadiansPerSecond{ 0.5f };
        accelerations[joint] = foc::RadiansPerSecondSquared{ 1.0f };
    }
    const auto dynamics = measure([this, &sink, &speeds, &accelerations](float q)
        {
            sink = model.InverseDynamics(Joints(q, 0.3f, 0.5f, q, 0.7f, q), speeds, accelerations)[1].Value();
        });
    const auto inertia = measure([this, &sink](float q)
        {
            sink = model.JointMassMatrix(Joints(q, 0.3f, 0.5f, q, 0.7f, q)).at(1, 1);
        });

    std::printf("[METRIC] arm model per call: tool pose %.2f us, jacobian %.2f us, singularities %.2f us, inverse %.2f us, gravity %.2f us, inverse dynamics %.2f us, "
                "mass matrix %.2f us\n",
        pose, jacobian, near, inverse, gravity, dynamics, inertia);
    for (const auto cost : { pose, jacobian, near, inverse, gravity, dynamics, inertia })
        EXPECT_LT(cost, 2000.0);
}
