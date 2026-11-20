#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "core/motion/implementations/JointMotionImpl.hpp"
#include "core/safety/test_doubles/PoseGuardMock.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <limits>
#include <vector>

namespace
{
    using namespace testing;
    using namespace std::chrono_literals;

    constexpr float tolerance{ 1.0e-4f };

    class JointMotionObserverMock
        : public arm::JointMotionObserver
    {
    public:
        using arm::JointMotionObserver::JointMotionObserver;

        MOCK_METHOD(void, OnMoveDone, (), (override));
        MOCK_METHOD(void, OnMoveAborted, (), (override));
    };

    arm::JointLimits PositionLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -2.0f }, foc::Radians{ 2.0f } };
        return limits;
    }

    arm::JointMotionLimits UniformLimits()
    {
        arm::JointMotionLimits limits{};
        for (auto& limit : limits)
            limit = { foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecondSquared{ 1.0f } };
        return limits;
    }

    class JointMotionImplTest
        : public Test
    {
    public:
        JointMotionImplTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                readings[joint].position = foc::Radians{ 0.1f * static_cast<float>(joint) };
                readings[joint].period = currentPeriod;
                EXPECT_CALL(bus, Reading(joint)).WillRepeatedly(ReturnRef(readings[joint]));
            }
            EXPECT_CALL(bus, Period()).WillRepeatedly(Invoke([this]()
                {
                    return currentPeriod;
                }));
        }

        arm::JointPositions Target(float firstJoint) const
        {
            arm::JointPositions target{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                target[joint] = readings[joint].position;
            target[0] = foc::Radians{ firstJoint };
            return target;
        }

        void Step(std::size_t periods)
        {
            for (std::size_t period = 0; period != periods; ++period)
                if (const auto reference = motion.Step())
                    sent.push_back(reference->positions);
        }

        StrictMock<arm::JointBusMock> bus;
        std::array<arm::JointReading, arm::jointCount> readings{};
        uint32_t currentPeriod{ 5 };
        arm::JointMotionImpl motion{ bus, PositionLimits(), UniformLimits(), 100ms };
        StrictMock<JointMotionObserverMock> observer{ motion };
        std::vector<arm::JointPositions> sent;
    };
}

TEST_F(JointMotionImplTest, samples_one_reference_per_period_and_the_target_last)
{
    const auto target = Target(1.0f);

    EXPECT_TRUE(motion.MoveTo(target));
    EXPECT_TRUE(motion.Moving());

    Step(15);
    EXPECT_EQ(15u, sent.size());
    EXPECT_TRUE(motion.Moving());

    EXPECT_CALL(observer, OnMoveDone());
    Step(5);

    ASSERT_EQ(20u, sent.size());
    EXPECT_EQ(target, sent.back());
    EXPECT_FALSE(motion.Moving());

    Step(10);
    EXPECT_EQ(20u, sent.size());
}

TEST_F(JointMotionImplTest, starts_from_the_latest_reported_positions)
{

    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));
    Step(1);

    ASSERT_EQ(1u, sent.size());
    EXPECT_NEAR(0.005f, sent.front()[0].Value(), tolerance);
    for (std::size_t joint = 1; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(readings[joint].position.Value(), sent.front()[joint].Value(), tolerance);
}

TEST_F(JointMotionImplTest, plans_from_readings_at_most_one_period_old)
{
    readings[3].period = currentPeriod - 1;

    EXPECT_TRUE(motion.MoveTo(Target(0.5f)));
}

TEST_F(JointMotionImplTest, refuses_to_plan_from_a_stale_reading)
{
    readings[3].period = currentPeriod - 2;
    EXPECT_FALSE(motion.MoveTo(Target(0.5f)));

    readings[3].period = 0;
    EXPECT_FALSE(motion.MoveTo(Target(0.5f)));
    EXPECT_FALSE(motion.Moving());
}

TEST_F(JointMotionImplTest, refuses_a_target_that_is_not_finite)
{
    EXPECT_FALSE(motion.MoveTo(Target(std::numeric_limits<float>::infinity())));

    EXPECT_FALSE(motion.Moving());
    Step(10);
}

TEST_F(JointMotionImplTest, refuses_a_target_outside_the_joint_limits)
{
    EXPECT_FALSE(motion.MoveTo(Target(2.01f)));

    EXPECT_FALSE(motion.Moving());
    Step(10);
}

TEST_F(JointMotionImplTest, refuses_a_second_move_while_moving)
{
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));

    EXPECT_FALSE(motion.MoveTo(Target(-1.0f)));

    EXPECT_CALL(observer, OnMoveDone());
    Step(20);
    EXPECT_EQ(Target(1.0f), sent.back());
}

TEST_F(JointMotionImplTest, abort_stops_sampling_and_reports_it_once)
{
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));
    Step(3);

    EXPECT_CALL(observer, OnMoveAborted());
    motion.Abort();
    motion.Abort();

    Step(20);
    EXPECT_EQ(3u, sent.size());
    EXPECT_FALSE(motion.Moving());
}

TEST_F(JointMotionImplTest, a_move_to_where_the_joints_are_samples_the_target_once_and_is_done)
{
    EXPECT_TRUE(motion.MoveTo(Target(0.0f)));

    EXPECT_CALL(observer, OnMoveDone());
    Step(1);

    ASSERT_EQ(1u, sent.size());
    EXPECT_EQ(Target(0.0f), sent.front());
}

TEST_F(JointMotionImplTest, starts_trapezoidal_and_refuses_the_s_curve_without_a_jerk_limit_on_every_joint)
{
    EXPECT_EQ(arm::MotionProfile::trapezoid, motion.Profile());

    EXPECT_FALSE(motion.SelectProfile(arm::MotionProfile::sCurve));
    EXPECT_EQ(arm::MotionProfile::trapezoid, motion.Profile());
}

TEST_F(JointMotionImplTest, a_step_without_a_move_has_no_reference)
{
    Step(5);

    EXPECT_TRUE(sent.empty());
    EXPECT_FALSE(motion.Moving());
}

TEST_F(JointMotionImplTest, a_step_jumps_the_reference_to_its_target_at_rest_and_is_done_at_once)
{
    EXPECT_TRUE(motion.StepTo(Target(0.15f)));
    EXPECT_TRUE(motion.Moving());

    EXPECT_CALL(observer, OnMoveDone());
    const auto reference = motion.Step();

    ASSERT_TRUE(reference.has_value());
    EXPECT_EQ(arm::AtRest(Target(0.15f)), *reference);
    EXPECT_FALSE(motion.Moving());
    Step(5);
    EXPECT_TRUE(sent.empty());
}

TEST_F(JointMotionImplTest, refuses_a_step_larger_than_0_2_rad_on_any_joint)
{
    EXPECT_FALSE(motion.StepTo(Target(0.21f)));
    EXPECT_FALSE(motion.StepTo(Target(-0.21f)));

    EXPECT_FALSE(motion.Moving());
}

TEST_F(JointMotionImplTest, refuses_a_step_outside_the_joint_limits_from_a_stale_reading_or_while_moving)
{
    readings[0].position = foc::Radians{ 1.9f };
    EXPECT_FALSE(motion.StepTo(Target(2.05f)));

    readings[0].position = foc::Radians{ 0.0f };
    readings[3].period = currentPeriod - 2;
    EXPECT_FALSE(motion.StepTo(Target(0.1f)));

    readings[3].period = currentPeriod;
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));
    EXPECT_FALSE(motion.StepTo(Target(0.1f)));
}

namespace
{
    class JointMotionGuardedTest
        : public JointMotionImplTest
    {
    public:
        StrictMock<arm::PoseGuardMock> guard;
        arm::JointMotionImpl guarded{ bus, PositionLimits(), UniformLimits(), 100ms, &guard };
    };
}

TEST_F(JointMotionGuardedTest, checks_the_path_every_0_05_rad_of_the_largest_travel_up_to_the_target)
{
    EXPECT_CALL(guard, Check(_)).Times(10).WillRepeatedly(Return(std::nullopt));

    EXPECT_TRUE(guarded.MoveTo(Target(0.5f)));
}

TEST_F(JointMotionGuardedTest, refuses_a_move_whose_path_fails_the_guard_and_never_starts_it)
{
    EXPECT_CALL(guard, Check(_)).WillOnce(Return(std::nullopt)).WillOnce(Return(std::nullopt)).WillOnce(Return(arm::WorkspaceViolation{ arm::WorkspaceRule::forbiddenZone, 3, 0, 0.01f }));

    EXPECT_FALSE(guarded.MoveTo(Target(0.5f)));
    EXPECT_FALSE(guarded.Moving());
}

TEST_F(JointMotionGuardedTest, checks_at_most_64_samples_on_a_long_move)
{
    readings[0].position = foc::Radians{ 2.0f };
    EXPECT_CALL(guard, Check(_)).Times(64).WillRepeatedly(Return(std::nullopt));

    EXPECT_TRUE(guarded.MoveTo(Target(-2.0f)));
}

TEST_F(JointMotionGuardedTest, refuses_a_step_whose_target_fails_the_guard)
{
    EXPECT_CALL(guard, Check(_)).WillOnce(Return(arm::WorkspaceViolation{ arm::WorkspaceRule::selfCollision, 0, 2, 0.0f }));

    EXPECT_FALSE(guarded.StepTo(Target(0.1f)));
}

namespace
{
    arm::JointJerkLimits Jerks()
    {
        arm::JointJerkLimits jerks{};
        jerks.fill(foc::RadiansPerSecondCubed{ 4.0f });
        return jerks;
    }

    class JointMotionJerkLimitedTest
        : public JointMotionImplTest
    {
    public:
        arm::JointMotionImpl jerkLimited{ bus, PositionLimits(), UniformLimits(), 100ms, nullptr, Jerks() };
    };
}

TEST_F(JointMotionJerkLimitedTest, switches_profile_at_rest_only_and_moves_with_the_one_in_effect_when_planned)
{
    EXPECT_TRUE(jerkLimited.SelectProfile(arm::MotionProfile::sCurve));
    EXPECT_EQ(arm::MotionProfile::sCurve, jerkLimited.Profile());

    ASSERT_TRUE(jerkLimited.MoveTo(Target(1.0f)));
    EXPECT_FALSE(jerkLimited.SelectProfile(arm::MotionProfile::trapezoid));
    EXPECT_EQ(arm::MotionProfile::sCurve, jerkLimited.Profile());

    const auto first = jerkLimited.Step();
    ASSERT_TRUE(first.has_value());
    EXPECT_NEAR(4.0f * 0.1f, first->accelerations[0].Value(), tolerance);
}
