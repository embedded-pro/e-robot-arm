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

        arm::ArmModel model{ arm::ReferenceGeometry(), FullTurn() };
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
    const arm::ArmModel placed{ geometry, FullTurn() };

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
    const arm::ArmModel narrow{ arm::ReferenceGeometry(), limits };
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

    std::printf("[METRIC] arm model per call: tool pose %.2f us, jacobian %.2f us, singularities %.2f us, inverse %.2f us\n", pose, jacobian, near, inverse);
    for (const auto cost : { pose, jacobian, near, inverse })
        EXPECT_LT(cost, 2000.0);
}
