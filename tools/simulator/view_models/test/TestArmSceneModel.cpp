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
