#include "core/motion/implementations/JointPathPlan.hpp"
#include "gmock/gmock.h"
#include <cmath>

namespace
{
    constexpr float tolerance{ 1.0e-3f };

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

    float Distance(const arm::JointPositions& a, const arm::JointPositions& b)
    {
        float largest{ 0.0f };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            largest = std::max(largest, std::abs(a[joint].Value() - b[joint].Value()));
        return largest;
    }

    class JointPathPlanTest
        : public testing::Test
    {
    public:
        void Waypoint(std::array<float, arm::jointCount> values)
        {
            waypoints.push_back(Positions(values));
        }

        float ClosestApproach(const arm::JointPositions& waypoint) const
        {
            float closest{ 1.0e9f };
            for (float time = 0.0f; time <= plan.Duration(); time += 1.0e-3f)
                closest = std::min(closest, Distance(waypoint, plan.Sample(time).positions));
            return closest;
        }

        static constexpr float period{ 0.01f };

        void ExpectEverySampleWithin(const arm::JointMotionLimits& limits) const
        {
            for (float time = period; time < plan.Duration(); time += period)
            {
                const auto sample = plan.Sample(time);
                for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                {
                    EXPECT_LE(std::abs(sample.speeds[joint].Value()), limits[joint].maxSpeed.Value()) << "joint " << joint << " at " << time;
                    EXPECT_LE(std::abs(sample.accelerations[joint].Value()), limits[joint].maxAcceleration.Value()) << "joint " << joint << " at " << time;
                }
            }
        }

        const arm::JointPositions start{ Positions({ 0.0f, 0.5f, -0.5f, 0.0f, 0.2f, 0.0f }) };
        arm::JointWaypoints::WithMaxSize<arm::maxWaypoints> waypoints;
        arm::JointPathPlan plan;
    };
}

TEST_F(JointPathPlanTest, runs_through_every_waypoint_from_rest_to_rest_at_the_last)
{
    Waypoint({ 1.0f, 0.0f, -0.5f, 0.5f, 0.2f, 0.0f });
    Waypoint({ 1.5f, -0.5f, 0.0f, 0.5f, -0.2f, 0.3f });
    Waypoint({ 0.5f, -0.5f, 0.5f, 0.0f, -0.2f, 0.3f });

    ASSERT_TRUE(plan.Plan(start, waypoints, UniformLimits(1.0f, 2.0f), period));

    EXPECT_GT(plan.Duration(), 0.0f);
    EXPECT_LT(Distance(start, plan.Sample(0.0f).positions), tolerance);
    EXPECT_LT(Distance(waypoints.back(), plan.Sample(plan.Duration()).positions), tolerance);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_NEAR(0.0f, plan.Sample(0.0f).speeds[joint].Value(), tolerance);
        EXPECT_NEAR(0.0f, plan.Sample(plan.Duration()).speeds[joint].Value(), tolerance);
    }
    for (const auto& waypoint : waypoints)
        EXPECT_LT(ClosestApproach(waypoint), 5.0e-3f);
}

TEST_F(JointPathPlanTest, no_joint_exceeds_its_speed_or_acceleration_limit_at_a_sample)
{
    Waypoint({ 1.0f, 0.0f, -0.5f, 0.5f, 0.2f, 0.0f });
    Waypoint({ 1.5f, -0.5f, 0.0f, 0.5f, -0.2f, 0.3f });
    auto limits = UniformLimits(1.0f, 2.0f);
    limits[1] = { foc::RadiansPerSecond{ 0.25f }, foc::RadiansPerSecondSquared{ 0.5f } };

    ASSERT_TRUE(plan.Plan(start, waypoints, limits, period));

    ExpectEverySampleWithin(limits);
}

TEST_F(JointPathPlanTest, a_path_that_turns_back_on_itself_stays_within_the_limits_at_every_sample)
{
    Waypoint({ 1.0f, 0.5f, -0.5f, 0.0f, 0.2f, 0.0f });
    Waypoint({ -0.5f, 0.5f, -0.5f, 0.0f, 0.2f, 0.0f });
    Waypoint({ 0.8f, 0.5f, -0.5f, 0.0f, 0.2f, 0.0f });
    const auto limits = UniformLimits(1.0f, 1.0f);

    ASSERT_TRUE(plan.Plan(start, waypoints, limits, period));

    ExpectEverySampleWithin(limits);
    EXPECT_LT(Distance(waypoints.back(), plan.Sample(plan.Duration()).positions), tolerance);
}

TEST_F(JointPathPlanTest, a_slower_joint_lengthens_the_path)
{
    Waypoint({ 1.0f, 0.0f, -0.5f, 0.5f, 0.2f, 0.0f });
    ASSERT_TRUE(plan.Plan(start, waypoints, UniformLimits(1.0f, 2.0f), period));
    const auto fast = plan.Duration();

    auto limits = UniformLimits(1.0f, 2.0f);
    limits[0] = { foc::RadiansPerSecond{ 0.25f }, foc::RadiansPerSecondSquared{ 2.0f } };
    ASSERT_TRUE(plan.Plan(start, waypoints, limits, period));

    EXPECT_GT(plan.Duration(), 1.5f * fast);
}

TEST_F(JointPathPlanTest, refuses_no_waypoints_and_a_waypoint_where_the_path_already_is)
{
    EXPECT_FALSE(plan.Plan(start, waypoints, UniformLimits(1.0f, 2.0f), period));

    waypoints.push_back(start);
    EXPECT_FALSE(plan.Plan(start, waypoints, UniformLimits(1.0f, 2.0f), period));

    waypoints.clear();
    Waypoint({ 1.0f, 0.0f, -0.5f, 0.5f, 0.2f, 0.0f });
    Waypoint({ 1.0f, 0.0f, -0.5f, 0.5f, 0.2f, 0.0f });
    EXPECT_FALSE(plan.Plan(start, waypoints, UniformLimits(1.0f, 2.0f), period));
}
