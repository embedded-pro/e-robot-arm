#include "tools/simulator/view_models/ArmSceneModel.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>

namespace
{
    class ArmSceneModelTest
        : public testing::Test
    {
    public:
        simulator::ArmSceneModel scene;
    };
}

TEST_F(ArmSceneModelTest, converts_measured_and_commanded_angles_to_degrees)
{
    auto snapshot = simulator::test::Running();
    snapshot.readings[0].position = foc::Radians{ std::numbers::pi_v<float> / 2.0f };
    snapshot.commanded = arm::JointPositions{};
    snapshot.commanded->at(0) = foc::Radians{ std::numbers::pi_v<float> };

    scene.Update(snapshot);

    EXPECT_NEAR(90.0f, scene.MeasuredDegrees()[0], 1.0e-3f);
    EXPECT_NEAR(180.0f, scene.CommandedDegrees()[0], 1.0e-3f);
}

TEST_F(ArmSceneModelTest, without_a_setpoint_the_ghost_sits_on_the_measured_arm_and_hides)
{
    auto snapshot = simulator::test::Running();
    snapshot.readings[2].position = foc::Radians{ 0.3f };

    scene.Update(snapshot);

    EXPECT_NEAR(scene.MeasuredDegrees()[2], scene.CommandedDegrees()[2], 1.0e-5f);
    EXPECT_FALSE(scene.GhostVisible());
}

TEST_F(ArmSceneModelTest, the_ghost_shows_while_moving_or_while_the_arms_differ)
{
    auto snapshot = simulator::test::Running();
    snapshot.commanded = arm::JointPositions{};
    snapshot.moving = true;
    scene.Update(snapshot);
    EXPECT_TRUE(scene.GhostVisible());

    snapshot.moving = false;
    snapshot.commanded->at(5) = foc::Radians{ 0.02f };
    scene.Update(snapshot);
    EXPECT_TRUE(scene.GhostVisible());

    snapshot.commanded->at(5) = foc::Radians{ 0.005f };
    scene.Update(snapshot);
    EXPECT_FALSE(scene.GhostVisible());
}

TEST_F(ArmSceneModelTest, selects_a_picked_joint_and_ignores_anything_else)
{
    EXPECT_TRUE(scene.Select(3));
    EXPECT_EQ(3u, scene.Selected());

    EXPECT_FALSE(scene.Select(6));
    EXPECT_FALSE(scene.Select(-1));
    EXPECT_EQ(3u, scene.Selected());

    scene.ClearSelection();
    EXPECT_FALSE(scene.Selected().has_value());
}

TEST_F(ArmSceneModelTest, status_levels_follow_the_joints_and_are_neutral_without_a_target)
{
    auto snapshot = simulator::test::Running();
    snapshot.readings[1].state = can::FocMotorState::fault;
    scene.Update(snapshot);
    EXPECT_EQ(ui::theme::StatusLevel::Fault, scene.Level(1));
    EXPECT_EQ(ui::theme::StatusLevel::Ok, scene.Level(0));

    snapshot.targetReady = false;
    scene.Update(snapshot);
    EXPECT_EQ(ui::theme::StatusLevel::Neutral, scene.Level(1));
}

TEST_F(ArmSceneModelTest, warns_while_the_arm_is_near_a_singularity)
{
    auto snapshot = simulator::test::Running();
    snapshot.singularities = { true, false, false };
    scene.Update(snapshot);
    EXPECT_EQ("Near wrist singularity", scene.SingularityWarning());

    snapshot.singularities = {};
    scene.Update(snapshot);
    EXPECT_TRUE(scene.SingularityWarning().empty());
}

TEST_F(ArmSceneModelTest, a_silent_joint_is_shown_at_fault_level)
{
    auto snapshot = simulator::test::Running();
    snapshot.readings[3].silent = true;

    scene.Update(snapshot);

    EXPECT_EQ(ui::theme::StatusLevel::Fault, scene.Level(3));
}

namespace
{
    std::array<float, 3> Rotate(const std::array<float, 4>& q, const std::array<float, 3>& v)
    {
        const auto [w, x, y, z] = q;
        const std::array<float, 3> t{ 2.0f * (y * v[2] - z * v[1]), 2.0f * (z * v[0] - x * v[2]), 2.0f * (x * v[1] - y * v[0]) };
        return { v[0] + w * t[0] + (y * t[2] - z * t[1]), v[1] + w * t[1] + (z * t[0] - x * t[2]), v[2] + w * t[2] + (x * t[1] - y * t[0]) };
    }

    std::array<float, 3> Place(const simulator::ScenePose& node, const std::array<float, 3>& local)
    {
        const auto rotated = Rotate(node.rotation, local);
        return { node.position[0] + rotated[0], node.position[1] + rotated[1], node.position[2] + rotated[2] };
    }

    arm_host::ArmSnapshot Placed()
    {
        auto snapshot = simulator::test::Running();
        auto& geometry = snapshot.config.geometry;
        geometry.opw.a2 = 0.03f;
        geometry.opw.b = 0.04f;
        geometry.opw.signs = { 1, -1, 1, -1, 1, -1 };
        geometry.opw.offsets = { 0.1f, -0.2f, 0.3f, 0.0f, 0.2f, -0.1f };
        geometry.base = { 0.2f, -0.1f, 0.05f, 0.0f, 0.0f, 0.7f };
        const std::array<float, arm::jointCount> joints{ 0.4f, -0.3f, 0.8f, 0.5f, -0.7f, 0.2f };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            snapshot.readings[joint].position = foc::Radians{ joints[joint] };
        return snapshot;
    }
}

TEST_F(ArmSceneModelTest, the_flange_sits_where_the_arm_model_puts_it_in_y_up_centimetres)
{
    const auto snapshot = Placed();
    scene.Update(snapshot);

    const auto& geometry = snapshot.config.geometry;
    const arm::ArmModel model{ geometry, snapshot.config.mass, snapshot.config.positionLimits };
    arm::JointPositions joints{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        joints[joint] = snapshot.readings[joint].position;
    const auto flange = model.ToolPose(joints).translation;

    const auto placed = Place(scene.MeasuredNodes()[6], { 0.0f, 0.0f, geometry.opw.c4 * 100.0f });
    EXPECT_NEAR(100.0f * flange[0], placed[0], 1.0e-3f);
    EXPECT_NEAR(100.0f * flange[2], placed[1], 1.0e-3f);
    EXPECT_NEAR(-100.0f * flange[1], placed[2], 1.0e-3f);

    const auto up = Rotate(scene.MeasuredNodes()[0].rotation, { 0.0f, 0.0f, 1.0f });
    EXPECT_NEAR(1.0f, up[1], 1.0e-5f);
    EXPECT_NEAR(20.0f, scene.MeasuredNodes()[0].position[0], 1.0e-4f);
    EXPECT_NEAR(10.0f, scene.MeasuredNodes()[0].position[2], 1.0e-4f);
}

TEST_F(ArmSceneModelTest, the_ghost_nodes_follow_the_commanded_joints)
{
    auto snapshot = Placed();
    snapshot.commanded = arm::JointPositions{};
    scene.Update(snapshot);

    EXPECT_NE(scene.MeasuredNodes()[3], scene.CommandedNodes()[3]);
    EXPECT_EQ(scene.MeasuredNodes()[0], scene.CommandedNodes()[0]);
}

TEST_F(ArmSceneModelTest, the_meshes_follow_the_geometry_and_are_rebuilt_only_when_its_lengths_change)
{
    auto snapshot = simulator::test::Running();
    scene.Update(snapshot);
    const auto revision = scene.MeshesRevision();
    EXPECT_NEAR(73.0f, scene.Extent(), 1.0e-3f);
    EXPECT_EQ(simulator::BuildDesktopArm(snapshot.config.geometry), scene.Meshes());

    snapshot.readings[1].position = foc::Radians{ 0.7f };
    snapshot.config.geometry.base.yaw = 0.3f;
    scene.Update(snapshot);
    EXPECT_EQ(revision, scene.MeshesRevision());

    snapshot.config.geometry.opw.c2 = 0.4f;
    scene.Update(snapshot);
    EXPECT_EQ(revision + 1, scene.MeshesRevision());
    EXPECT_EQ(simulator::BuildDesktopArm(snapshot.config.geometry), scene.Meshes());
}

TEST_F(ArmSceneModelTest, the_tool_node_sits_at_the_tool_centre_point)
{
    auto snapshot = Placed();
    snapshot.config.geometry.tool = { 0.0f, 0.01f, 0.08f, 0.2f, 0.0f, 0.0f };
    scene.Update(snapshot);

    arm::JointPositions joints{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        joints[joint] = snapshot.readings[joint].position;
    const arm::ArmModel model{ snapshot.config.geometry, snapshot.config.mass, snapshot.config.positionLimits };
    EXPECT_EQ(simulator::ToScene(model.ToolPose(joints)), scene.ToolNode());
}

TEST_F(ArmSceneModelTest, the_trail_adds_spaced_points_keeps_the_newest_and_clears_when_the_arm_changes)
{
    auto snapshot = simulator::test::Running();
    scene.Update(snapshot);
    scene.Update(snapshot);
    EXPECT_EQ(1u, scene.ToolTrail().size());

    for (std::size_t step = 1; step != simulator::ArmSceneModel::trailCapacity + 50; ++step)
    {
        snapshot.readings[1].position = foc::Radians{ 0.005f * static_cast<float>(step) };
        scene.Update(snapshot);
    }
    EXPECT_EQ(simulator::ArmSceneModel::trailCapacity, scene.ToolTrail().size());
    EXPECT_EQ(scene.ToolNode().position, scene.ToolTrail().back());

    const auto revision = scene.TrailRevision();
    snapshot.readings[2].silent = true;
    scene.Update(snapshot);
    EXPECT_TRUE(scene.ToolTrail().empty());
    EXPECT_NE(revision, scene.TrailRevision());

    snapshot.readings[2].silent = false;
    scene.Update(snapshot);
    snapshot.config.geometry.opw.c2 = 0.3f;
    scene.Update(snapshot);
    EXPECT_EQ(1u, scene.ToolTrail().size());
}

TEST_F(ArmSceneModelTest, draws_the_workspace_and_each_forbidden_zone_in_y_up_centimetres)
{
    auto snapshot = simulator::test::Running();
    snapshot.workspace = { math::Vector3<float>{ -0.5f, -0.4f, 0.0f }, math::Vector3<float>{ 0.5f, 0.4f, 0.9f } };
    snapshot.config.workspace.forbiddenZones[0] = { math::Vector3<float>{ 0.2f, 0.1f, 0.0f }, math::Vector3<float>{ 0.4f, 0.3f, 0.2f } };
    snapshot.config.workspace.forbiddenZoneCount = 1;

    scene.Update(snapshot);

    const auto& boxes = scene.Boxes();
    ASSERT_EQ(2u, boxes.size());
    EXPECT_EQ(simulator::SceneBoxKind::workspace, boxes[0].kind);
    EXPECT_NEAR(0.0f, boxes[0].centre[0], 1.0e-4f);
    EXPECT_NEAR(45.0f, boxes[0].centre[1], 1.0e-4f);
    EXPECT_NEAR(100.0f, boxes[0].size[0], 1.0e-4f);
    EXPECT_NEAR(90.0f, boxes[0].size[1], 1.0e-4f);
    EXPECT_NEAR(80.0f, boxes[0].size[2], 1.0e-4f);
    EXPECT_EQ(simulator::SceneBoxKind::zone, boxes[1].kind);
    EXPECT_NEAR(30.0f, boxes[1].centre[0], 1.0e-4f);
    EXPECT_NEAR(10.0f, boxes[1].centre[1], 1.0e-4f);
    EXPECT_NEAR(-20.0f, boxes[1].centre[2], 1.0e-4f);
    EXPECT_FALSE(boxes[0].violated);
    EXPECT_FALSE(boxes[1].violated);
}

TEST_F(ArmSceneModelTest, marks_the_zone_a_protective_stop_names_and_rebuilds_the_boxes_only_on_change)
{
    auto snapshot = simulator::test::Running();
    snapshot.config.workspace.forbiddenZoneCount = 2;
    scene.Update(snapshot);
    const auto revision = scene.BoxesRevision();
    scene.Update(snapshot);
    EXPECT_EQ(revision, scene.BoxesRevision());

    snapshot.holding = true;
    snapshot.stop = arm_host::StopReport{ .category = arm_host::StopCategory::protective,
        .joint = 2,
        .reason = "forearm in forbidden zone 2 by 0.010 m",
        .workspace = arm::WorkspaceViolation{ .rule = arm::WorkspaceRule::forbiddenZone, .link = 3, .other = 1, .distance = 0.01f } };
    scene.Update(snapshot);

    EXPECT_NE(revision, scene.BoxesRevision());
    EXPECT_FALSE(scene.Boxes()[0].violated);
    EXPECT_FALSE(scene.Boxes()[1].violated);
    EXPECT_TRUE(scene.Boxes()[2].violated);
}

TEST_F(ArmSceneModelTest, marks_the_workspace_box_on_a_stop_by_the_workspace_rule)
{
    auto snapshot = simulator::test::Running();
    snapshot.holding = true;
    snapshot.stop = arm_host::StopReport{ .category = arm_host::StopCategory::protective,
        .joint = 0,
        .workspace = arm::WorkspaceViolation{ .rule = arm::WorkspaceRule::workspace, .link = 6, .distance = 0.02f } };

    scene.Update(snapshot);

    EXPECT_TRUE(scene.Boxes()[0].violated);
}

TEST_F(ArmSceneModelTest, a_box_outline_runs_along_all_twelve_edges)
{
    const simulator::SceneBox box{ simulator::SceneBoxKind::zone, { 10.0f, 20.0f, 30.0f }, { 2.0f, 4.0f, 6.0f }, false };

    const auto path = simulator::BoxOutline(box);

    std::set<std::pair<std::array<float, 3>, std::array<float, 3>>> edges;
    for (std::size_t point = 1; point != path.size(); ++point)
    {
        const auto& a = path[point - 1];
        const auto& b = path[point];
        int differing = 0;
        for (std::size_t axis = 0; axis != 3; ++axis)
        {
            EXPECT_NEAR(box.size[axis] / 2.0f, std::abs(a[axis] - box.centre[axis]), 1.0e-5f);
            differing += a[axis] != b[axis] ? 1 : 0;
        }
        EXPECT_EQ(1, differing);
        edges.insert(std::minmax(a, b));
    }
    EXPECT_EQ(12u, edges.size());
}

TEST_F(ArmSceneModelTest, the_target_marker_sits_at_the_tool_pose_of_the_move_targets_and_hides_on_the_measured_tool)
{
    const auto snapshot = simulator::test::Running();
    arm::JointPositions target{};
    target[1] = foc::Radians{ 0.4f };

    scene.SetTarget(target);
    scene.Update(snapshot);

    ASSERT_TRUE(scene.TargetNode().has_value());
    const arm::ArmModel model{ snapshot.config.geometry, snapshot.config.mass, snapshot.config.positionLimits };
    const auto expected = simulator::ToScene(model.ToolPose(target));
    for (std::size_t axis = 0; axis != 3; ++axis)
        EXPECT_NEAR(expected.position[axis], scene.TargetNode()->position[axis], 1.0e-3f);

    scene.SetTarget(arm::JointPositions{});
    scene.Update(snapshot);
    EXPECT_FALSE(scene.TargetNode().has_value());

    scene.SetTarget(std::nullopt);
    scene.Update(snapshot);
    EXPECT_FALSE(scene.TargetNode().has_value());
}

TEST_F(ArmSceneModelTest, a_selected_joint_shows_an_arc_over_its_range_around_its_axis)
{
    auto snapshot = simulator::test::Running();
    snapshot.config.positionLimits[0] = { foc::Radians{ -1.0f }, foc::Radians{ 1.5f } };
    scene.Update(snapshot);
    EXPECT_TRUE(scene.LimitArc().empty());
    const auto before = scene.LimitArcRevision();

    ASSERT_TRUE(scene.Select(0));
    scene.Update(snapshot);

    const auto& arc = scene.LimitArc();
    ASSERT_EQ(simulator::ArmSceneModel::limitArcPoints, arc.size());
    EXPECT_NE(before, scene.LimitArcRevision());
    const auto& centre = scene.MeasuredNodes()[1].position;
    const auto radius = 0.08f * scene.Extent();
    for (const auto& point : arc)
    {
        EXPECT_NEAR(radius, std::hypot(point[0] - centre[0], point[2] - centre[2]), 1.0e-2f);
        EXPECT_NEAR(centre[1], point[1], 1.0e-3f);
    }
    const auto sweep = std::atan2(-arc.back()[2] + centre[2], arc.back()[0] - centre[0]) - std::atan2(-arc.front()[2] + centre[2], arc.front()[0] - centre[0]);
    EXPECT_NEAR(2.5f, sweep, 1.0e-2f);

    scene.ClearSelection();
    scene.Update(snapshot);
    EXPECT_TRUE(scene.LimitArc().empty());
}
