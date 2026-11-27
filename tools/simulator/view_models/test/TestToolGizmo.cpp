#include "tools/simulator/view_models/ToolGizmo.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <variant>
#include <vector>

namespace
{
    using namespace std::chrono_literals;
    using simulator::GizmoHandle;
    using simulator::ToolGizmo;
    using Vector = math::Vector3<float>;

    class ToolGizmoTest
        : public testing::Test
    {
    public:
        ToolGizmoTest()
        {
            snapshot = simulator::test::Running();
            snapshot.disturbances = arm_host::Disturbances{};
            snapshot.config.positionLimits = simulator::test::Limits();
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                snapshot.readings[joint].position = foc::Radians{ pose[joint] };
        }

        arm::JointPositions Joints() const
        {
            arm::JointPositions joints{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                joints[joint] = foc::Radians{ pose[joint] };
            return joints;
        }

        arm::ArmModel::Pose Tool() const
        {
            return model.ToolPose(Joints());
        }

        Vector ToolAxis(std::size_t axis) const
        {
            const auto tool = Tool();
            return Vector{ tool.rotation.at(0, axis), tool.rotation.at(1, axis), tool.rotation.at(2, axis) };
        }

        // A ray through a base-frame point, looking along -direction.
        static simulator::SceneRay Through(const Vector& point, const Vector& direction)
        {
            const auto from = point + direction * 2.0f;
            return { simulator::ToScenePoint(from), simulator::ToScenePoint(direction * -1.0f) };
        }

        static simulator::SceneRay FromAbove(const Vector& point)
        {
            return Through(point, Vector{ 0.0f, 0.0f, 1.0f });
        }

        bool GrabMoveX(bool follow = false)
        {
            return gizmo.Grab(GizmoHandle::moveX, FromAbove(Tool().translation), snapshot, move, follow);
        }

        void MoveXBy(float metres, ToolGizmo::Clock::time_point at = {})
        {
            gizmo.Drag(FromAbove(Tool().translation + Vector{ metres, 0.0f, 0.0f }), snapshot, move, at);
        }

        template<class Command>
        bool Posted(std::size_t index) const
        {
            return index < posted.size() && std::holds_alternative<Command>(posted[index]);
        }

        std::array<float, arm::jointCount> pose{ 0.2f, 0.4f, 0.5f, 0.3f, 0.8f, -0.2f };
        arm_host::ArmSnapshot snapshot;
        simulator::ControlState move{ true, {} };
        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), simulator::test::Limits() };
        std::vector<arm_host::ArmCommand> posted;
        ToolGizmo gizmo{ [this](const arm_host::ArmCommand& command)
            {
                posted.push_back(command);
            } };
    };
}

TEST_F(ToolGizmoTest, grabs_only_on_a_connected_target_and_posts_nothing)
{
    auto disconnected = snapshot;
    disconnected.targetReady = false;
    EXPECT_FALSE(gizmo.Grab(GizmoHandle::moveX, FromAbove(Tool().translation), disconnected, move, false));
    EXPECT_FALSE(gizmo.Dragging());

    EXPECT_TRUE(GrabMoveX());
    EXPECT_TRUE(gizmo.Dragging());
    EXPECT_EQ(GizmoHandle::moveX, gizmo.Handle());
    EXPECT_FALSE(gizmo.Target().has_value());
    EXPECT_TRUE(posted.empty());
}

TEST_F(ToolGizmoTest, a_ray_along_the_arrow_cannot_grab_it)
{
    EXPECT_FALSE(gizmo.Grab(GizmoHandle::moveX, Through(Tool().translation, Vector{ 1.0f, 0.0f, 0.0f }), snapshot, move, false));
    EXPECT_FALSE(gizmo.Dragging());
}

TEST_F(ToolGizmoTest, an_arrow_moves_the_tool_along_its_base_axis_and_keeps_its_orientation)
{
    ASSERT_TRUE(GrabMoveX());
    MoveXBy(-0.05f);

    ASSERT_TRUE(gizmo.Target().has_value());
    EXPECT_TRUE(gizmo.Reachable());
    const auto reached = model.ToolPose(*gizmo.Target());
    const auto expected = Tool().translation + Vector{ -0.05f, 0.0f, 0.0f };
    for (std::size_t axis = 0; axis != 3; ++axis)
        EXPECT_NEAR(expected[axis], reached.translation[axis], 1.0e-3f);
    for (std::size_t row = 0; row != 3; ++row)
        for (std::size_t column = 0; column != 3; ++column)
            EXPECT_NEAR(Tool().rotation.at(row, column), reached.rotation.at(row, column), 1.0e-3f);
    EXPECT_TRUE(posted.empty());
}

TEST_F(ToolGizmoTest, a_ring_turns_the_tool_about_its_own_axis_and_keeps_its_position)
{
    const auto centre = Tool().translation;
    const auto axis = ToolAxis(2);
    const auto reference = ToolAxis(0);
    const auto side = ToolAxis(1);
    ASSERT_TRUE(gizmo.Grab(GizmoHandle::turnZ, Through(centre + reference * 0.1f, axis), snapshot, move, false));
    const auto angle = std::numbers::pi_v<float> / 6.0f;
    gizmo.Drag(Through(centre + (reference * std::cos(angle) + side * std::sin(angle)) * 0.1f, axis), snapshot, move, {});

    ASSERT_TRUE(gizmo.Target().has_value());
    const auto reached = model.ToolPose(*gizmo.Target());
    const auto expected = math::RotationAboutAxis(axis, angle) * Tool().rotation;
    for (std::size_t row = 0; row != 3; ++row)
    {
        EXPECT_NEAR(centre[row], reached.translation[row], 1.0e-3f);
        for (std::size_t column = 0; column != 3; ++column)
            EXPECT_NEAR(expected.at(row, column), reached.rotation.at(row, column), 2.0e-3f);
    }
    EXPECT_THAT(gizmo.Label(move), testing::HasSubstr("Δ 0.0 cm 30°"));
}

TEST_F(ToolGizmoTest, the_label_gives_the_target_and_its_distance_from_the_tool)
{
    ASSERT_TRUE(GrabMoveX());
    MoveXBy(-0.05f);
    gizmo.Release();

    const auto to = Tool().translation + Vector{ -0.05f, 0.0f, 0.0f };
    EXPECT_EQ(std::format("Tool → x {:.3f}  y {:.3f}  z {:.3f} m · Δ 5.0 cm 0° · Enter: move · Del: clear", to[0], to[1], to[2]), gizmo.Label(move));
    EXPECT_EQ(ui::theme::StatusLevel::Neutral, gizmo.Level());
    ASSERT_TRUE(gizmo.TargetNodes().has_value());
    ASSERT_TRUE(gizmo.TargetPoint().has_value());
    EXPECT_NEAR(to[0] * 100.0f, (*gizmo.TargetPoint())[0], 1.0e-2f);
}

TEST_F(ToolGizmoTest, out_of_reach_keeps_the_last_reachable_pose_and_says_so)
{
    ASSERT_TRUE(GrabMoveX());
    MoveXBy(-0.05f);
    const auto reachable = *gizmo.Target();
    MoveXBy(5.0f);

    EXPECT_FALSE(gizmo.Reachable());
    EXPECT_EQ(reachable, *gizmo.Target());
    EXPECT_THAT(gizmo.Label(move), testing::HasSubstr(" · out of reach"));
    EXPECT_EQ(ui::theme::StatusLevel::Fault, gizmo.Level());

    gizmo.Release();
    EXPECT_TRUE(gizmo.Confirm(move));
    ASSERT_TRUE(Posted<arm_host::MoveTo>(0));
    EXPECT_EQ(reachable, std::get<arm_host::MoveTo>(posted[0]).target);
}

TEST_F(ToolGizmoTest, a_target_near_a_singularity_is_a_warning)
{
    pose[4] = 0.0f;
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        snapshot.readings[joint].position = foc::Radians{ pose[joint] };
    const auto centre = Tool().translation;
    ASSERT_TRUE(gizmo.Grab(GizmoHandle::turnZ, Through(centre + ToolAxis(0) * 0.1f, ToolAxis(2)), snapshot, move, false));
    gizmo.Drag(Through(centre + (ToolAxis(0) + ToolAxis(1) * 0.2f) * 0.1f, ToolAxis(2)), snapshot, move, {});

    EXPECT_TRUE(gizmo.Reachable());
    EXPECT_THAT(gizmo.Label(move), testing::HasSubstr(" · near wrist"));
    EXPECT_EQ(ui::theme::StatusLevel::Warning, gizmo.Level());
}

TEST_F(ToolGizmoTest, follow_moves_only_to_reachable_targets_and_aborts_on_release)
{
    ASSERT_TRUE(GrabMoveX(true));
    const ToolGizmo::Clock::time_point start{};
    MoveXBy(-0.02f, start);
    MoveXBy(5.0f, start + 100ms);
    MoveXBy(-0.03f, start + 200ms);

    ASSERT_EQ(2u, posted.size());
    EXPECT_TRUE(Posted<arm_host::MoveTo>(0));
    EXPECT_TRUE(Posted<arm_host::MoveTo>(1));
    EXPECT_THAT(gizmo.Label(move), testing::EndsWith(" · following"));

    gizmo.Release();
    ASSERT_EQ(3u, posted.size());
    EXPECT_TRUE(Posted<arm_host::AbortMove>(2));
    EXPECT_FALSE(gizmo.Target().has_value());
    EXPECT_FALSE(gizmo.Handle().has_value());
}

TEST_F(ToolGizmoTest, delete_clears_the_target)
{
    ASSERT_TRUE(GrabMoveX());
    MoveXBy(-0.05f);
    gizmo.Release();
    gizmo.Cancel();

    EXPECT_FALSE(gizmo.Target().has_value());
    EXPECT_EQ("", gizmo.Label(move));
    EXPECT_FALSE(gizmo.Confirm(move));
    EXPECT_TRUE(posted.empty());
}

TEST_F(ToolGizmoTest, a_grab_that_misses_keeps_the_earlier_target)
{
    ASSERT_TRUE(GrabMoveX());
    MoveXBy(-0.05f);
    gizmo.Release();
    const auto label = gizmo.Label(move);

    EXPECT_FALSE(gizmo.Grab(GizmoHandle::moveY, Through(Tool().translation, Vector{ 0.0f, 1.0f, 0.0f }), snapshot, move, false));
    EXPECT_EQ(label, gizmo.Label(move));
    EXPECT_EQ(GizmoHandle::moveX, gizmo.Handle());
}

TEST(GizmoHandlesTest, arrows_run_from_the_tool_along_the_base_axes_and_rings_circle_it)
{
    arm::ArmModel::Pose at{};
    at.translation = Vector{ 0.3f, 0.0f, 0.4f };
    const auto lines = simulator::GizmoLines(at, 0.1f);

    const auto tip = simulator::ToScenePoint(Vector{ 0.3f, 0.1f, 0.4f });
    ASSERT_EQ(5u, lines[static_cast<std::size_t>(GizmoHandle::moveY)].size());
    EXPECT_EQ(simulator::ToScenePoint(at.translation), lines[static_cast<std::size_t>(GizmoHandle::moveY)][0]);
    for (std::size_t axis = 0; axis != 3; ++axis)
        EXPECT_NEAR(tip[axis], lines[static_cast<std::size_t>(GizmoHandle::moveY)][1][axis], 1.0e-4f);

    const auto& ring = lines[static_cast<std::size_t>(GizmoHandle::turnZ)];
    ASSERT_EQ(simulator::gizmoRingPoints + 1, ring.size());
    const auto centre = simulator::ToScenePoint(at.translation);
    for (const auto& point : ring)
    {
        EXPECT_NEAR(8.0f, std::hypot(point[0] - centre[0], point[2] - centre[2]), 1.0e-3f);
        EXPECT_NEAR(centre[1], point[1], 1.0e-3f);
    }
}

TEST(GizmoHandlesTest, a_ray_picks_the_arrow_or_ring_it_passes_and_nothing_else)
{
    arm::ArmModel::Pose at{};
    at.translation = Vector{ 0.3f, 0.0f, 0.4f };
    const auto down = [](const Vector& point)
    {
        return simulator::SceneRay{ simulator::ToScenePoint(point + Vector{ 0.0f, 0.0f, 1.0f }), simulator::ToScenePoint(Vector{ 0.0f, 0.0f, -1.0f }) };
    };

    EXPECT_EQ(GizmoHandle::moveX, simulator::GizmoHandleAt(down(at.translation + Vector{ 0.05f, 0.0f, 0.0f }), at, 0.1f));
    const auto onRing = 0.08f * std::numbers::sqrt2_v<float> / 2.0f;
    EXPECT_EQ(GizmoHandle::turnZ, simulator::GizmoHandleAt(down(at.translation + Vector{ onRing, onRing, 0.0f }), at, 0.1f));
    EXPECT_FALSE(simulator::GizmoHandleAt(down(at.translation + Vector{ 0.05f, 0.05f, 0.0f }), at, 0.1f).has_value());
    EXPECT_FALSE(simulator::GizmoHandleAt(down(at.translation + Vector{ 0.15f, 0.0f, 0.0f }), at, 0.1f).has_value());
}

TEST_F(ToolGizmoTest, the_gizmo_follows_the_target_pose_while_one_is_set)
{
    EXPECT_FALSE(gizmo.WantedPose().has_value());
    ASSERT_TRUE(GrabMoveX());
    MoveXBy(-0.05f);
    gizmo.Release();

    ASSERT_TRUE(gizmo.WantedPose().has_value());
    EXPECT_NEAR(Tool().translation[0] - 0.05f, gizmo.WantedPose()->translation[0], 1.0e-4f);
    gizmo.Cancel();
    EXPECT_FALSE(gizmo.WantedPose().has_value());
}
