#include "tools/simulator/view_models/ArmSceneModel.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"
#include <numbers>

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

TEST_F(ArmSceneModelTest, the_parts_are_sized_by_the_geometry_and_rebuilt_only_when_it_changes)
{
    auto snapshot = simulator::test::Running();
    scene.Update(snapshot);
    const auto revision = scene.PartsRevision();

    const auto& upperArm = scene.Parts()[2];
    ASSERT_EQ(2u, upperArm.size());
    EXPECT_EQ(1, upperArm[1].joint);
    EXPECT_NEAR(snapshot.config.geometry.opw.c2, upperArm[1].scale[1], 1.0e-6f);
    const auto top = Rotate(upperArm[1].rotation, { 0.0f, 50.0f * upperArm[1].scale[1], 0.0f });
    EXPECT_NEAR(100.0f * snapshot.config.geometry.opw.c2, upperArm[1].centre[2] + top[2], 1.0e-3f);
    EXPECT_EQ(-1, scene.Parts()[0].front().joint);
    EXPECT_NEAR(73.0f, scene.Extent(), 1.0e-3f);

    snapshot.readings[1].position = foc::Radians{ 0.7f };
    scene.Update(snapshot);
    EXPECT_EQ(revision, scene.PartsRevision());

    snapshot.config.geometry.opw.c2 = 0.4f;
    scene.Update(snapshot);
    EXPECT_EQ(revision + 1, scene.PartsRevision());
    EXPECT_NEAR(0.4f, scene.Parts()[2][1].scale[1], 1.0e-6f);
}
