#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "core/motion/implementations/JointMotionImpl.hpp"
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
                motion.Step();
        }

        void RecordSetpoints(bool accepted = true)
        {
            EXPECT_CALL(bus, SendPositions(_)).WillRepeatedly(Invoke([this, accepted](const arm::JointPositions& positions)
                {
                    sent.push_back(positions);
                    return accepted;
                }));
        }

        StrictMock<arm::JointBusMock> bus;
        std::array<arm::JointReading, arm::jointCount> readings{};
        uint32_t currentPeriod{ 5 };
        arm::JointMotionImpl motion{ bus, PositionLimits(), UniformLimits(), 100ms };
        StrictMock<JointMotionObserverMock> observer{ motion };
        std::vector<arm::JointPositions> sent;
    };
}

TEST_F(JointMotionImplTest, streams_one_setpoint_set_per_period_and_the_target_last)
{
    const auto target = Target(1.0f);
    RecordSetpoints();

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
    RecordSetpoints();

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
    RecordSetpoints();
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));

    EXPECT_FALSE(motion.MoveTo(Target(-1.0f)));

    EXPECT_CALL(observer, OnMoveDone());
    Step(20);
    EXPECT_EQ(Target(1.0f), sent.back());
}

TEST_F(JointMotionImplTest, a_refused_setpoint_set_aborts_the_move)
{
    RecordSetpoints(false);
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));

    EXPECT_CALL(observer, OnMoveAborted());
    Step(20);

    EXPECT_EQ(1u, sent.size());
    EXPECT_FALSE(motion.Moving());
}

TEST_F(JointMotionImplTest, abort_stops_streaming_and_reports_it_once)
{
    RecordSetpoints();
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));
    Step(3);

    EXPECT_CALL(observer, OnMoveAborted());
    motion.Abort();
    motion.Abort();

    Step(20);
    EXPECT_EQ(3u, sent.size());
    EXPECT_FALSE(motion.Moving());
}

TEST_F(JointMotionImplTest, a_move_to_where_the_joints_are_sends_the_target_once_and_is_done)
{
    RecordSetpoints();
    EXPECT_TRUE(motion.MoveTo(Target(0.0f)));

    EXPECT_CALL(observer, OnMoveDone());
    Step(1);

    ASSERT_EQ(1u, sent.size());
    EXPECT_EQ(Target(0.0f), sent.front());
}

TEST_F(JointMotionImplTest, a_step_without_a_move_sends_nothing)
{
    Step(5);

    EXPECT_FALSE(motion.Moving());
}

TEST_F(JointMotionImplTest, has_no_setpoint_before_the_first_move)
{
    EXPECT_FALSE(motion.Setpoint().has_value());
}

TEST_F(JointMotionImplTest, reports_the_last_accepted_setpoint_set)
{
    RecordSetpoints();
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));
    Step(3);

    ASSERT_TRUE(motion.Setpoint().has_value());
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(motion.Setpoint()->at(joint).Value(), sent.back()[joint].Value(), tolerance);
}

TEST_F(JointMotionImplTest, keeps_the_last_setpoint_after_the_move_is_done)
{
    RecordSetpoints();
    EXPECT_CALL(observer, OnMoveDone());
    EXPECT_TRUE(motion.MoveTo(Target(0.0f)));
    Step(1);

    ASSERT_TRUE(motion.Setpoint().has_value());
    EXPECT_NEAR(motion.Setpoint()->at(0).Value(), 0.0f, tolerance);
}

TEST_F(JointMotionImplTest, a_refused_setpoint_set_is_not_reported)
{
    RecordSetpoints(false);
    EXPECT_CALL(observer, OnMoveAborted());
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));
    Step(1);

    EXPECT_FALSE(motion.Setpoint().has_value());
}
