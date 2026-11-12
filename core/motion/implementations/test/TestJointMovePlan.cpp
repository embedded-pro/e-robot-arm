#include "core/motion/implementations/JointMovePlan.hpp"
#include "gmock/gmock.h"
#include <algorithm>
#include <cmath>

namespace
{
    constexpr float tolerance{ 1.0e-4f };

    arm::JointPositions Positions(std::array<float, arm::jointCount> values)
    {
        arm::JointPositions positions{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            positions[joint] = foc::Radians{ values[joint] };
        return positions;
    }

    arm::JointMotionLimits UniformLimits(float maxSpeed, float maxAcceleration)
    {
        arm::JointMotionLimits limits{};
        for (auto& limit : limits)
            limit = { foc::RadiansPerSecond{ maxSpeed }, foc::RadiansPerSecondSquared{ maxAcceleration } };
        return limits;
    }

    class JointMovePlanTest
        : public testing::Test
    {
    public:
        void ExpectPositions(const arm::JointPositions& expected, const arm::JointPositions& actual) const
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                EXPECT_NEAR(expected[joint].Value(), actual[joint].Value(), tolerance) << "joint " << joint;
        }

        const arm::JointPositions from{ Positions({ 0.0f, 1.0f, -1.0f, 0.5f, 0.0f, 2.0f }) };
        const arm::JointPositions to{ Positions({ 2.0f, 0.5f, -1.0f, 1.0f, -0.25f, 1.0f }) };
        arm::JointMovePlan plan;
    };
}

TEST_F(JointMovePlanTest, the_slowest_joint_sets_the_duration)
{
    plan.Plan(from, to, UniformLimits(1.0f, 1.0f));

    EXPECT_NEAR(3.0f, plan.Duration(), tolerance);
}

TEST_F(JointMovePlanTest, a_joint_with_lower_limits_can_be_the_slowest)
{
    auto limits = UniformLimits(1.0f, 1.0f);
    limits[1] = { foc::RadiansPerSecond{ 0.1f }, foc::RadiansPerSecondSquared{ 1.0f } };

    plan.Plan(from, to, limits);

    EXPECT_NEAR(5.1f, plan.Duration(), tolerance);
}

TEST_F(JointMovePlanTest, all_joints_start_together_and_arrive_together)
{
    plan.Plan(from, to, UniformLimits(1.0f, 1.0f));

    ExpectPositions(from, plan.Sample(0.0f));
    ExpectPositions(to, plan.Sample(plan.Duration()));
    ExpectPositions(to, plan.Sample(plan.Duration() + 1.0f));
}

TEST_F(JointMovePlanTest, every_joint_is_halfway_at_half_the_duration)
{
    plan.Plan(from, to, UniformLimits(1.0f, 1.0f));

    arm::JointPositions halfway{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        halfway[joint] = foc::Radians{ (from[joint].Value() + to[joint].Value()) / 2.0f };

    ExpectPositions(halfway, plan.Sample(plan.Duration() / 2.0f));
}

TEST_F(JointMovePlanTest, no_joint_exceeds_its_speed_limit)
{
    constexpr float maxSpeed{ 1.0f };
    constexpr float step{ 1.0e-3f };
    plan.Plan(from, to, UniformLimits(maxSpeed, 1.0f));

    auto previous = plan.Sample(0.0f);
    for (float t = step; t <= plan.Duration(); t += step)
    {
        const auto current = plan.Sample(t);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            EXPECT_LE(std::abs(current[joint].Value() - previous[joint].Value()) / step, maxSpeed + 1.0e-2f) << "joint " << joint << " at " << t;
        previous = current;
    }
}

TEST_F(JointMovePlanTest, a_move_to_where_the_joints_are_takes_no_time)
{
    plan.Plan(from, from, UniformLimits(1.0f, 1.0f));

    EXPECT_NEAR(0.0f, plan.Duration(), tolerance);
    ExpectPositions(from, plan.Sample(0.0f));
}
