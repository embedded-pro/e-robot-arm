#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "core/motion/implementations/JointMotionImpl.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
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

    arm::JointMotionLimits UniformLimits()
    {
        arm::JointMotionLimits limits{};
        for (auto& limit : limits)
            limit = { foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecondSquared{ 1.0f } };
        return limits;
    }

    class JointMotionImplTest
        : public Test
        , public infra::ClockFixture
    {
    public:
        JointMotionImplTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                readings[joint].position = foc::Radians{ 0.1f * static_cast<float>(joint) };
                EXPECT_CALL(bus, Reading(joint)).WillRepeatedly(ReturnRef(readings[joint]));
            }
        }

        arm::JointPositions Target(float firstJoint) const
        {
            arm::JointPositions target{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                target[joint] = readings[joint].position;
            target[0] = foc::Radians{ firstJoint };
            return target;
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
        arm::JointMotionImpl motion{ bus, UniformLimits(), 100ms };
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

    ForwardTime(1500ms);
    EXPECT_EQ(15u, sent.size());
    EXPECT_TRUE(motion.Moving());

    EXPECT_CALL(observer, OnMoveDone());
    ForwardTime(500ms);

    ASSERT_EQ(20u, sent.size());
    EXPECT_EQ(target, sent.back());
    EXPECT_FALSE(motion.Moving());

    ForwardTime(1s);
    EXPECT_EQ(20u, sent.size());
}

TEST_F(JointMotionImplTest, starts_from_the_latest_reported_positions)
{
    RecordSetpoints();

    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));
    ForwardTime(100ms);

    ASSERT_EQ(1u, sent.size());
    EXPECT_NEAR(0.005f, sent.front()[0].Value(), tolerance);
    for (std::size_t joint = 1; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(readings[joint].position.Value(), sent.front()[joint].Value(), tolerance);
}

TEST_F(JointMotionImplTest, refuses_a_target_that_is_not_finite)
{
    EXPECT_FALSE(motion.MoveTo(Target(std::numeric_limits<float>::infinity())));

    EXPECT_FALSE(motion.Moving());
    ForwardTime(1s);
}

TEST_F(JointMotionImplTest, refuses_a_second_move_while_moving)
{
    RecordSetpoints();
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));

    EXPECT_FALSE(motion.MoveTo(Target(-1.0f)));

    EXPECT_CALL(observer, OnMoveDone());
    ForwardTime(2s);
    EXPECT_EQ(Target(1.0f), sent.back());
}

TEST_F(JointMotionImplTest, a_refused_setpoint_set_aborts_the_move)
{
    RecordSetpoints(false);
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));

    EXPECT_CALL(observer, OnMoveAborted());
    ForwardTime(2s);

    EXPECT_EQ(1u, sent.size());
    EXPECT_FALSE(motion.Moving());
}

TEST_F(JointMotionImplTest, abort_stops_streaming_and_reports_it_once)
{
    RecordSetpoints();
    EXPECT_TRUE(motion.MoveTo(Target(1.0f)));
    ForwardTime(300ms);

    EXPECT_CALL(observer, OnMoveAborted());
    motion.Abort();
    motion.Abort();

    ForwardTime(2s);
    EXPECT_EQ(3u, sent.size());
    EXPECT_FALSE(motion.Moving());
}

TEST_F(JointMotionImplTest, a_move_to_where_the_joints_are_sends_the_target_once_and_is_done)
{
    RecordSetpoints();
    EXPECT_TRUE(motion.MoveTo(Target(0.0f)));

    EXPECT_CALL(observer, OnMoveDone());
    ForwardTime(100ms);

    ASSERT_EQ(1u, sent.size());
    EXPECT_EQ(Target(0.0f), sent.front());
}
