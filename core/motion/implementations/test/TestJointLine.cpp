#include "core/arm_model/ArmModel.hpp"
#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "core/motion/implementations/JointMotionImpl.hpp"
#include "core/safety/test_doubles/PoseGuardMock.hpp"
#include "numerical/motion_profiles/SCurveProfile.hpp"
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

    class JointLineTest
        : public Test
    {
    public:
        JointLineTest()
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

        arm::ToolPose Shifted(float x, float y, float z) const
        {
            auto pose = model.ToolPose(Start());
            pose.translation[0] += x;
            pose.translation[1] += y;
            pose.translation[2] += z;
            return pose;
        }

        std::vector<arm::JointReference> Run(arm::JointMotionImpl& motion)
        {
            std::vector<arm::JointReference> references;
            while (motion.Moving())
                if (const auto reference = motion.Step())
                    references.push_back(*reference);
            return references;
        }

        StrictMock<arm::JointBusMock> bus;
        std::array<arm::JointReading, arm::jointCount> readings{};
        const arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), FullTurn() };
        arm::ToolLimits tool{};
        arm::JointMotionImpl motion{ bus, FullTurn(), UniformLimits(5.0f, 50.0f), 10ms, nullptr, {}, arm::ToolLineSetup{ &model, tool } };
        StrictMock<JointMotionObserverMock> observer{ motion };
    };
}

TEST_F(JointLineTest, moves_the_tool_along_a_straight_line_and_is_done_at_rest_at_the_target)
{
    const auto from = model.ToolPose(Start()).translation;
    const auto target = Shifted(0.0f, 0.04f, -0.02f);

    ASSERT_TRUE(motion.MoveLine(target));
    EXPECT_FALSE(motion.LineRefused().has_value());

    EXPECT_CALL(observer, OnMoveDone());
    const auto references = Run(motion);
    ASSERT_FALSE(references.empty());
    const auto direction = target.translation - from;
    const auto length = Distance(target.translation, from);
    for (const auto& reference : references)
    {
        const auto at = model.ToolPose(reference.positions).translation - from;
        const auto along = (at[0] * direction[0] + at[1] * direction[1] + at[2] * direction[2]) / length;
        const auto across = std::sqrt(std::max(0.0f, at[0] * at[0] + at[1] * at[1] + at[2] * at[2] - along * along));
        EXPECT_LT(across, 1.0e-3f);
        EXPECT_GE(along, -1.0e-3f);
        EXPECT_LE(along, length + 1.0e-3f);
    }
    EXPECT_LT(Distance(target.translation, model.ToolPose(references.back().positions).translation), 1.0e-3f);
    EXPECT_EQ(arm::JointSpeeds{}, references.back().speeds);
}

TEST_F(JointLineTest, takes_as_long_as_the_tool_limits_allow_for_its_length)
{
    const motion_profiles::SCurveProfile<float> alone{ 0.0f, 0.05f, { tool.speedMPerS, tool.accelerationMPerS2, tool.jerkMPerS3 } };

    ASSERT_TRUE(motion.MoveLine(Shifted(0.0f, 0.05f, 0.0f)));

    EXPECT_CALL(observer, OnMoveDone());
    const auto periods = Run(motion).size();
    EXPECT_EQ(static_cast<std::size_t>(std::ceil(alone.Duration() / 0.01f)), periods);
}

TEST_F(JointLineTest, refuses_an_unreachable_target_and_names_the_first_failing_sample)
{
    tool.manipulabilityMin = 0.0f;
    arm::JointMotionImpl fast{ bus, FullTurn(), UniformLimits(1.0e6f, 1.0e9f), 10ms, nullptr, {}, arm::ToolLineSetup{ &model, tool } };

    EXPECT_FALSE(fast.MoveLine(Shifted(2.0f, 0.0f, 0.0f)));

    ASSERT_TRUE(fast.LineRefused().has_value());
    EXPECT_EQ(arm::LineFault::unreachable, fast.LineRefused()->fault);
    EXPECT_GT(fast.LineRefused()->timeS, 0.01f);
    EXPECT_FALSE(fast.Moving());
}

TEST_F(JointLineTest, refuses_a_line_that_straightens_the_elbow_as_too_fast_for_a_joint)
{
    EXPECT_FALSE(motion.MoveLine(Shifted(0.05f, 0.0f, 0.0f)));

    ASSERT_TRUE(motion.LineRefused().has_value());
    EXPECT_EQ(arm::LineFault::tooFast, motion.LineRefused()->fault);
}

TEST_F(JointLineTest, refuses_a_line_too_fast_for_a_joint)
{
    arm::JointMotionImpl slow{ bus, FullTurn(), UniformLimits(0.01f, 50.0f), 10ms, nullptr, {}, arm::ToolLineSetup{ &model, tool } };

    EXPECT_FALSE(slow.MoveLine(Shifted(0.0f, 0.05f, 0.0f)));

    ASSERT_TRUE(slow.LineRefused().has_value());
    EXPECT_EQ(arm::LineFault::tooFast, slow.LineRefused()->fault);
}

TEST_F(JointLineTest, refuses_a_line_below_the_manipulability_threshold)
{
    arm::ToolLimits strict{};
    strict.manipulabilityMin = 1.0f;
    arm::JointMotionImpl careful{ bus, FullTurn(), UniformLimits(5.0f, 50.0f), 10ms, nullptr, {}, arm::ToolLineSetup{ &model, strict } };

    EXPECT_FALSE(careful.MoveLine(Shifted(0.0f, 0.05f, 0.0f)));

    ASSERT_TRUE(careful.LineRefused().has_value());
    EXPECT_EQ((arm::LineRefusal{ arm::LineFault::nearSingularity, 0.01f }), *careful.LineRefused());
}

TEST_F(JointLineTest, refuses_a_line_that_fails_the_workspace_guard)
{
    StrictMock<arm::PoseGuardMock> guard;
    arm::JointMotionImpl guarded{ bus, FullTurn(), UniformLimits(5.0f, 50.0f), 10ms, &guard, {}, arm::ToolLineSetup{ &model, tool } };
    EXPECT_CALL(guard, Check(_)).WillOnce(Return(std::nullopt)).WillOnce(Return(arm::WorkspaceViolation{}));

    EXPECT_FALSE(guarded.MoveLine(Shifted(0.0f, 0.05f, 0.0f)));

    ASSERT_TRUE(guarded.LineRefused().has_value());
    EXPECT_EQ(arm::LineFault::workspace, guarded.LineRefused()->fault);
    EXPECT_NEAR(0.02f, guarded.LineRefused()->timeS, 1.0e-6f);
}

TEST_F(JointLineTest, a_line_runs_alone_and_a_line_to_where_the_tool_is_is_done_at_once)
{
    ASSERT_TRUE(motion.MoveLine(Shifted(0.0f, 0.05f, 0.0f)));
    EXPECT_FALSE(motion.MoveTo(Start()));
    EXPECT_FALSE(motion.MoveLine(Shifted(0.0f, -0.05f, 0.0f)));
    EXPECT_CALL(observer, OnMoveAborted());
    motion.Abort();

    ASSERT_TRUE(motion.MoveLine(model.ToolPose(Start())));
    EXPECT_CALL(observer, OnMoveDone());
    EXPECT_EQ(1u, Run(motion).size());
}

TEST_F(JointLineTest, refuses_a_line_without_an_arm_model)
{
    arm::JointMotionImpl blind{ bus, FullTurn(), UniformLimits(5.0f, 50.0f), 10ms };

    EXPECT_FALSE(blind.MoveLine(Shifted(0.0f, 0.05f, 0.0f)));
    EXPECT_FALSE(blind.LineRefused().has_value());
}
