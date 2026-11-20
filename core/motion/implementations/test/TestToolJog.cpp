#include "core/arm_model/ArmModel.hpp"
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

    class JointMotionObserverMock
        : public arm::JointMotionObserver
    {
    public:
        using arm::JointMotionObserver::JointMotionObserver;

        MOCK_METHOD(void, OnMoveDone, (), (override));
        MOCK_METHOD(void, OnMoveAborted, (), (override));
    };

    arm::JointLimits FullTurn()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        return limits;
    }

    arm::JointMotionLimits UniformLimits(float speed, float acceleration)
    {
        arm::JointMotionLimits limits{};
        for (auto& limit : limits)
            limit = { foc::RadiansPerSecond{ speed }, foc::RadiansPerSecondSquared{ acceleration } };
        return limits;
    }

    float Distance(const math::Vector3<float>& a, const math::Vector3<float>& b)
    {
        const auto d = a - b;
        return std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    }

    float RotationDifference(const math::Matrix3<float>& a, const math::Matrix3<float>& b)
    {
        float largest{ 0.0f };
        for (std::size_t row = 0; row != 3; ++row)
            for (std::size_t column = 0; column != 3; ++column)
                largest = std::max(largest, std::abs(a.at(row, column) - b.at(row, column)));
        return largest;
    }

    class ToolJogTest
        : public Test
    {
    public:
        ToolJogTest()
        {
            const std::array<float, arm::jointCount> start{ 0.1f, 0.5f, 0.6f, 0.2f, 1.0f, 0.0f };
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                readings[joint].position = foc::Radians{ start[joint] };
                readings[joint].period = 5;
                EXPECT_CALL(bus, Reading(joint)).WillRepeatedly(ReturnRef(readings[joint]));
            }
            EXPECT_CALL(bus, Period()).WillRepeatedly(Return(5));
        }

        arm::JointPositions Start() const
        {
            arm::JointPositions positions{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                positions[joint] = readings[joint].position;
            return positions;
        }

        void Hold(arm::JointMotionImpl& jogging, const arm::ToolJog& jog, std::size_t periods)
        {
            for (std::size_t period = 0; period != periods && jogging.Moving(); ++period)
            {
                jogging.Jog(jog);
                if (const auto reference = jogging.Step())
                    sent.push_back(*reference);
            }
        }

        void RunOut(arm::JointMotionImpl& jogging)
        {
            while (jogging.Moving())
                if (const auto reference = jogging.Step())
                    sent.push_back(*reference);
        }

        StrictMock<arm::JointBusMock> bus;
        std::array<arm::JointReading, arm::jointCount> readings{};
        const arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), FullTurn() };
        arm::ToolLimits tool{};
        arm::JointMotionImpl motion{ bus, FullTurn(), UniformLimits(5.0f, 50.0f), 10ms, nullptr, {}, arm::ToolLineSetup{ &model, tool } };
        StrictMock<JointMotionObserverMock> observer{ motion };
        std::vector<arm::JointReference> sent;
    };
}

TEST_F(ToolJogTest, moves_the_tool_along_a_base_axis_ramping_at_the_tool_acceleration)
{
    const auto start = model.ToolPose(Start());
    const arm::ToolJog jog{ arm::JogFrame::base, arm::ToolAxis::y, 0.1f };
    ASSERT_TRUE(motion.Jog(jog));

    Hold(motion, jog, 30);

    const auto end = model.ToolPose(sent.back().positions);
    EXPECT_NEAR(start.translation[0], end.translation[0], 1.0e-3f);
    EXPECT_NEAR(start.translation[2], end.translation[2], 1.0e-3f);
    EXPECT_NEAR(start.translation[1] + 0.0255f, end.translation[1], 1.0e-3f);
    EXPECT_LT(RotationDifference(start.rotation, end.rotation), 1.0e-3f);
}

TEST_F(ToolJogTest, turns_about_a_tool_axis_through_the_tool_point)
{
    const auto start = model.ToolPose(Start());
    const arm::ToolJog jog{ arm::JogFrame::tool, arm::ToolAxis::rz, 0.5f };
    ASSERT_TRUE(motion.Jog(jog));

    Hold(motion, jog, 20);

    const auto end = model.ToolPose(sent.back().positions);
    EXPECT_LT(Distance(start.translation, end.translation), 1.0e-3f);
    EXPECT_GT(RotationDifference(start.rotation, end.rotation), 0.05f);
}

TEST_F(ToolJogTest, ramps_to_rest_and_is_done_once_the_dead_man_expires)
{
    ASSERT_TRUE(motion.Jog(arm::ToolJog{ arm::JogFrame::base, arm::ToolAxis::z, -0.05f }));

    EXPECT_CALL(observer, OnMoveDone());
    RunOut(motion);
    EXPECT_GT(sent.size(), arm::jogDeadManPeriods);
    for (const auto& speed : sent.back().speeds)
        EXPECT_NEAR(0.0f, speed.Value(), 1.0e-3f);
}

TEST_F(ToolJogTest, slows_the_tool_so_no_joint_exceeds_its_speed_limit)
{
    arm::JointMotionImpl slow{ bus, FullTurn(), UniformLimits(0.2f, 50.0f), 10ms, nullptr, {}, arm::ToolLineSetup{ &model, tool } };
    const auto start = model.ToolPose(Start());
    const arm::ToolJog jog{ arm::JogFrame::base, arm::ToolAxis::y, 0.25f };
    ASSERT_TRUE(slow.Jog(jog));

    Hold(slow, jog, 40);

    for (const auto& reference : sent)
        for (const auto& speed : reference.speeds)
            EXPECT_LE(std::abs(speed.Value()), 0.2f + 1.0e-4f);
    EXPECT_GT(model.ToolPose(sent.back().positions).translation[1], start.translation[1] + 0.01f);
}

TEST_F(ToolJogTest, holds_short_of_straightening_the_elbow_within_every_joint_limit_until_released)
{
    const arm::ToolJog jog{ arm::JogFrame::base, arm::ToolAxis::x, 0.1f };
    ASSERT_TRUE(motion.Jog(jog));

    Hold(motion, jog, 300);
    EXPECT_TRUE(motion.Moving());
    for (const auto& speed : sent.back().speeds)
        EXPECT_LT(std::abs(speed.Value()), 0.05f);

    EXPECT_CALL(observer, OnMoveDone());
    RunOut(motion);
    for (const auto& reference : sent)
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            EXPECT_LE(std::abs(reference.speeds[joint].Value()), 5.0f);
            EXPECT_LE(std::abs(reference.accelerations[joint].Value()), 50.0f);
        }
    EXPECT_GE(model.Manipulability(sent.back().positions), tool.manipulabilityMin);
}

TEST_F(ToolJogTest, a_joint_jog_ramps_a_running_tool_jog_to_rest_and_then_starts)
{
    const arm::ToolJog jog{ arm::JogFrame::base, arm::ToolAxis::y, 0.1f };
    ASSERT_TRUE(motion.Jog(jog));
    Hold(motion, jog, 10);

    EXPECT_TRUE(motion.Jog(arm::JointJog{ 0, foc::RadiansPerSecond{ 0.2f } }));
    for (int period = 0; period != 15; ++period)
    {
        motion.Jog(arm::JointJog{ 0, foc::RadiansPerSecond{ 0.2f } });
        if (const auto reference = motion.Step())
            sent.push_back(*reference);
    }

    EXPECT_TRUE(motion.Moving());
    EXPECT_GT(sent.back().speeds[0].Value(), 0.0f);
    EXPECT_NEAR(0.0f, sent.back().speeds[1].Value(), 1.0e-4f);
}

TEST_F(ToolJogTest, refuses_a_tool_jog_without_an_arm_model_a_speed_or_while_moving)
{
    arm::JointMotionImpl blind{ bus, FullTurn(), UniformLimits(5.0f, 50.0f), 10ms };
    EXPECT_FALSE(blind.Jog(arm::ToolJog{ arm::JogFrame::base, arm::ToolAxis::y, 0.1f }));
    EXPECT_FALSE(motion.Jog(arm::ToolJog{ arm::JogFrame::base, arm::ToolAxis::y, 0.0f }));
    EXPECT_FALSE(motion.Jog(arm::ToolJog{ arm::JogFrame::base, arm::ToolAxis::y, std::nanf("") }));

    auto target = Start();
    target[0] = foc::Radians{ 0.3f };
    ASSERT_TRUE(motion.MoveTo(target));
    EXPECT_FALSE(motion.Jog(arm::ToolJog{ arm::JogFrame::base, arm::ToolAxis::y, 0.1f }));
}
