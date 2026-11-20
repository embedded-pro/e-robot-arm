#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "core/motion/implementations/JointMotionImpl.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <cmath>
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
            limit = { foc::Radians{ -1.0f }, foc::Radians{ 1.0f } };
        return limits;
    }

    arm::JointMotionLimits UniformLimits()
    {
        arm::JointMotionLimits limits{};
        for (auto& limit : limits)
            limit = { foc::RadiansPerSecond{ 0.5f }, foc::RadiansPerSecondSquared{ 1.0f } };
        return limits;
    }

    arm::JointJog Jog(std::size_t joint, float speed)
    {
        return { joint, foc::RadiansPerSecond{ speed } };
    }

    class JointJogTest
        : public Test
    {
    public:
        JointJogTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                readings[joint].position = foc::Radians{ 0.1f * static_cast<float>(joint) };
                readings[joint].period = 5;
                EXPECT_CALL(bus, Reading(joint)).WillRepeatedly(ReturnRef(readings[joint]));
            }
            EXPECT_CALL(bus, Period()).WillRepeatedly(Return(5));
        }

        void Step(std::size_t periods)
        {
            for (std::size_t period = 0; period != periods; ++period)
                if (const auto reference = motion.Step())
                    sent.push_back(*reference);
        }

        StrictMock<arm::JointBusMock> bus;
        std::array<arm::JointReading, arm::jointCount> readings{};
        arm::JointMotionImpl motion{ bus, PositionLimits(), UniformLimits(), 100ms };
        StrictMock<JointMotionObserverMock> observer{ motion };
        std::vector<arm::JointReference> sent;
    };
}

TEST_F(JointJogTest, ramps_one_joint_up_at_its_acceleration_limit_while_the_others_hold)
{
    ASSERT_TRUE(motion.Jog(Jog(1, 0.3f)));
    EXPECT_TRUE(motion.Moving());

    Step(3);
    EXPECT_NEAR(0.1f, sent[0].speeds[1].Value(), tolerance);
    EXPECT_NEAR(1.0f, sent[0].accelerations[1].Value(), tolerance);
    EXPECT_NEAR(0.3f, sent[2].speeds[1].Value(), tolerance);
    EXPECT_NEAR(0.1f + 0.1f * (0.05f + 0.15f + 0.25f), sent[2].positions[1].Value(), tolerance);
    EXPECT_EQ(readings[0].position, sent[2].positions[0]);
    EXPECT_EQ(foc::RadiansPerSecond{}, sent[2].speeds[0]);
}

TEST_F(JointJogTest, clamps_the_commanded_speed_to_the_joints_speed_limit)
{
    ASSERT_TRUE(motion.Jog(Jog(2, -3.0f)));

    Step(8);
    EXPECT_NEAR(-0.5f, sent.back().speeds[2].Value(), tolerance);
}

TEST_F(JointJogTest, ramps_to_rest_and_is_done_once_the_dead_man_expires)
{
    ASSERT_TRUE(motion.Jog(Jog(0, 0.3f)));
    Step(arm::jogDeadManPeriods);
    EXPECT_NEAR(0.3f, sent.back().speeds[0].Value(), tolerance);

    EXPECT_CALL(observer, OnMoveDone());
    Step(3);
    EXPECT_NEAR(0.0f, sent.back().speeds[0].Value(), tolerance);
    EXPECT_FALSE(motion.Moving());
}

TEST_F(JointJogTest, each_jog_command_refreshes_the_dead_man)
{
    ASSERT_TRUE(motion.Jog(Jog(0, 0.1f)));
    for (int refresh = 0; refresh != 4; ++refresh)
    {
        Step(arm::jogDeadManPeriods - 1);
        EXPECT_TRUE(motion.Jog(Jog(0, 0.1f)));
    }

    EXPECT_NEAR(0.1f, sent.back().speeds[0].Value(), tolerance);
    EXPECT_TRUE(motion.Moving());
}

TEST_F(JointJogTest, a_jog_of_speed_zero_ramps_to_rest_and_ends_it)
{
    ASSERT_TRUE(motion.Jog(Jog(0, 0.3f)));
    Step(3);

    EXPECT_TRUE(motion.Jog(Jog(0, 0.0f)));
    EXPECT_CALL(observer, OnMoveDone());
    Step(3);
    EXPECT_FALSE(motion.Moving());
    EXPECT_NEAR(0.0f, sent.back().speeds[0].Value(), tolerance);
}

TEST_F(JointJogTest, stops_at_rest_within_the_position_limit_without_exceeding_a_limit)
{
    ASSERT_TRUE(motion.Jog(Jog(5, 0.5f)));

    EXPECT_CALL(observer, OnMoveDone());
    for (int period = 0; period != 60 && motion.Moving(); ++period)
    {
        motion.Jog(Jog(5, 0.5f));
        Step(1);
    }

    EXPECT_FALSE(motion.Moving());
    for (const auto& reference : sent)
    {
        EXPECT_LE(reference.positions[5].Value(), 1.0f);
        EXPECT_LE(std::abs(reference.speeds[5].Value()), 0.5f + tolerance);
        EXPECT_LE(std::abs(reference.accelerations[5].Value()), 1.0f + tolerance);
    }
    EXPECT_NEAR(0.0f, sent.back().speeds[5].Value(), tolerance);
    EXPECT_GT(sent.back().positions[5].Value(), 0.95f);
}

TEST_F(JointJogTest, a_jog_of_another_joint_ramps_the_running_one_to_rest_first)
{
    ASSERT_TRUE(motion.Jog(Jog(0, 0.2f)));
    Step(2);

    EXPECT_TRUE(motion.Jog(Jog(1, -0.2f)));
    Step(2);
    EXPECT_NEAR(0.0f, sent.back().speeds[0].Value(), tolerance);
    EXPECT_NEAR(0.0f, sent.back().speeds[1].Value(), tolerance);

    Step(1);
    EXPECT_NEAR(0.0f, sent.back().speeds[0].Value(), tolerance);
    EXPECT_NEAR(-0.1f, sent.back().speeds[1].Value(), tolerance);
    EXPECT_NEAR(sent[3].positions[0].Value(), sent.back().positions[0].Value(), tolerance);
}

TEST_F(JointJogTest, refuses_a_first_jog_of_speed_zero_of_no_joint_or_while_another_generator_runs)
{
    EXPECT_FALSE(motion.Jog(Jog(0, 0.0f)));
    EXPECT_FALSE(motion.Jog(Jog(6, 0.2f)));
    EXPECT_FALSE(motion.Jog(Jog(0, std::nanf(""))));

    arm::JointPositions target{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        target[joint] = readings[joint].position;
    target[0] = foc::Radians{ 0.5f };
    ASSERT_TRUE(motion.MoveTo(target));
    EXPECT_FALSE(motion.Jog(Jog(0, 0.2f)));
    EXPECT_CALL(observer, OnMoveAborted());
    motion.Abort();

    ASSERT_TRUE(motion.Jog(Jog(0, 0.2f)));
    EXPECT_FALSE(motion.MoveTo(target));
}
