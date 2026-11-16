#include "tools/simulator/view_models/VirtualMotorRows.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"

namespace
{
    using ui::theme::StatusLevel;
    using virtual_arm::MotorPhase;

    class VirtualMotorRowsTest
        : public testing::Test
    {
    public:
        VirtualMotorRowsTest()
        {
            snapshot.target = { arm_host::TargetKind::virtualArm, "virtual:arm", "e-foc-virtual-motor" };
            arm_host::VirtualMotorsStatus status{ {}, "/tmp/e-robot-arm-virtual-arm-7-1" };
            for (auto& motor : status.motors)
                motor = { MotorPhase::up, "teknic", std::nullopt };
            snapshot.virtualMotors = status;
        }

        arm_host::ArmSnapshot snapshot{ simulator::test::Compatible() };
    };
}

TEST_F(VirtualMotorRowsTest, shows_each_joints_motor_and_process_and_offers_a_restart)
{
    snapshot.virtualMotors->motors[1] = { MotorPhase::starting, "anaheim", std::nullopt };
    snapshot.virtualMotors->motors[2] = { MotorPhase::ended, "teknic", 3 };
    snapshot.virtualMotors->motors[3] = { MotorPhase::stopped, "teknic", std::nullopt };

    const auto view = simulator::ViewVirtualMotors(snapshot);

    EXPECT_TRUE(view.visible);
    EXPECT_EQ("J1", view.rows[0].joint);
    EXPECT_EQ("teknic", view.rows[0].motor);
    EXPECT_EQ("Up", view.rows[0].process);
    EXPECT_EQ(StatusLevel::Ok, view.rows[0].level);
    EXPECT_TRUE(view.rows[0].restartEnabled);

    EXPECT_EQ("anaheim", view.rows[1].motor);
    EXPECT_EQ("Starting…", view.rows[1].process);
    EXPECT_EQ(StatusLevel::Neutral, view.rows[1].level);
    EXPECT_FALSE(view.rows[1].restartEnabled);

    EXPECT_EQ("Ended: exit code 3 (see joint3.log)", view.rows[2].process);
    EXPECT_EQ(StatusLevel::Fault, view.rows[2].level);
    EXPECT_TRUE(view.rows[2].restartEnabled);

    EXPECT_EQ("Stopped", view.rows[3].process);
    EXPECT_EQ(StatusLevel::Neutral, view.rows[3].level);

    EXPECT_EQ("Logs and memory: /tmp/e-robot-arm-virtual-arm-7-1", view.folder);
    EXPECT_EQ("", view.hint);
}

TEST_F(VirtualMotorRowsTest, offers_no_restart_while_the_arm_runs_and_says_why)
{
    for (const auto state : { arm::ArmState::enabling, arm::ArmState::enabled, arm::ArmState::moving, arm::ArmState::disabling })
    {
        snapshot.state = state;
        const auto view = simulator::ViewVirtualMotors(snapshot);

        EXPECT_FALSE(view.rows[0].restartEnabled);
        EXPECT_EQ("Restart: disable the arm first", view.hint);
    }
}

TEST_F(VirtualMotorRowsTest, offers_no_restart_until_the_virtual_arm_is_up)
{
    snapshot.targetReady = false;
    const auto view = simulator::ViewVirtualMotors(snapshot);

    EXPECT_FALSE(view.rows[0].restartEnabled);
    EXPECT_EQ("Restart: the virtual arm is starting", view.hint);
}

TEST_F(VirtualMotorRowsTest, shows_placeholders_before_the_motors_are_known_and_hides_on_other_targets)
{
    snapshot.virtualMotors.reset();
    auto view = simulator::ViewVirtualMotors(snapshot);
    EXPECT_TRUE(view.visible);
    EXPECT_EQ("—", view.rows[4].motor);
    EXPECT_EQ("—", view.rows[4].process);
    EXPECT_FALSE(view.rows[4].restartEnabled);
    EXPECT_EQ("", view.folder);

    snapshot.target = { arm_host::TargetKind::simulated, "" };
    view = simulator::ViewVirtualMotors(snapshot);
    EXPECT_FALSE(view.visible);
}

TEST_F(VirtualMotorRowsTest, counts_the_motors_up_while_the_target_starts)
{
    snapshot.targetReady = false;
    snapshot.virtualMotors->motors[4].phase = MotorPhase::starting;
    snapshot.virtualMotors->motors[5].phase = MotorPhase::stopped;
    EXPECT_EQ("Starting virtual motors: 4 of 6 up", simulator::StartingVirtualMotors(snapshot));

    snapshot.virtualMotors.reset();
    EXPECT_EQ("Starting virtual motors: 0 of 6 up", simulator::StartingVirtualMotors(snapshot));
}
