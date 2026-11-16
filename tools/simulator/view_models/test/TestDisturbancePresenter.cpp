#include "tools/simulator/view_models/DisturbancePresenter.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "ui/backend/recording/RecordingFormView.hpp"
#include "gmock/gmock.h"
#include <vector>

namespace
{
    using simulator::DisturbanceAction;
    using virtual_arm::MotorPhase;

    class DisturbancePresenterTest
        : public testing::Test
    {
    public:
        arm_host::ArmSnapshot Simulated() const
        {
            auto snapshot = simulator::test::Compatible();
            snapshot.disturbances = arm_host::Disturbances{ .injectFault = true };
            return snapshot;
        }

        arm_host::ArmSnapshot Virtual() const
        {
            auto snapshot = simulator::test::Compatible();
            snapshot.target = { arm_host::TargetKind::virtualArm, "virtual:arm", "e-foc-virtual-motor" };
            snapshot.disturbances = arm_host::Disturbances{ .collision = true, .powerOff = true };
            snapshot.virtualMotors = arm_host::VirtualMotorsStatus{};
            for (auto& motor : snapshot.virtualMotors->motors)
                motor.phase = MotorPhase::up;
            return snapshot;
        }

        ui::backend::recording::RecordingFormView view;
        std::vector<arm_host::ArmCommand> posted;
        simulator::DisturbancePresenter presenter{ view, [this](const arm_host::ArmCommand& command)
            {
                posted.push_back(command);
            } };
    };
}

TEST_F(DisturbancePresenterTest, is_hidden_and_offers_nothing_outside_a_simulation)
{
    presenter.Refresh(simulator::test::Compatible());

    EXPECT_FALSE(presenter.Visible());
    EXPECT_FALSE(presenter.Enabled(DisturbanceAction::silence));
    EXPECT_THAT(presenter.Hint(), testing::HasSubstr("Silence: not a simulation"));
    view.PressAction(DisturbanceAction::silence);
    EXPECT_TRUE(posted.empty());
}

TEST_F(DisturbancePresenterTest, offers_faults_and_silence_on_simulated_joints_but_no_torque)
{
    presenter.Refresh(Simulated());

    EXPECT_TRUE(presenter.Visible());
    EXPECT_TRUE(presenter.Enabled(DisturbanceAction::injectFault));
    EXPECT_TRUE(presenter.Enabled(DisturbanceAction::silence));
    EXPECT_FALSE(presenter.Enabled(DisturbanceAction::resume));
    EXPECT_FALSE(presenter.Enabled(DisturbanceAction::applyTorque));
    EXPECT_FALSE(presenter.Enabled(DisturbanceAction::powerOff));
    EXPECT_THAT(presenter.Hint(), testing::HasSubstr("Apply Torque: not on simulated joints"));
    EXPECT_THAT(presenter.Hint(), testing::HasSubstr("Resume: J1 is answering"));

    presenter.SelectJoint(2);
    view.PressAction(DisturbanceAction::injectFault);
    view.PressAction(DisturbanceAction::silence);
    ASSERT_EQ(2u, posted.size());
    EXPECT_EQ(2u, std::get<arm_host::InjectFault>(posted[0]).joint);
    EXPECT_EQ(2u, std::get<arm_host::SilenceJoint>(posted[1]).joint);
}

TEST_F(DisturbancePresenterTest, offers_torque_and_power_off_on_the_virtual_arm_and_lists_what_is_applied)
{
    auto snapshot = Virtual();
    snapshot.disturbances->silent[1] = true;
    snapshot.disturbances->torque[2] = foc::NewtonMeter{ 30.0f };
    presenter.Refresh(snapshot);

    EXPECT_FALSE(presenter.Enabled(DisturbanceAction::injectFault));
    EXPECT_THAT(presenter.Hint(), testing::HasSubstr("Inject Fault: not on the virtual arm"));
    EXPECT_EQ("Active: J2 silent · J3 30.00 N·m", presenter.Active());

    presenter.SelectJoint(1);
    EXPECT_FALSE(presenter.Enabled(DisturbanceAction::silence));
    EXPECT_TRUE(presenter.Enabled(DisturbanceAction::resume));
    EXPECT_FALSE(presenter.Enabled(DisturbanceAction::clearTorque));
    EXPECT_THAT(presenter.Hint(), testing::HasSubstr("Clear Torque: no torque on J2"));
    view.PressAction(DisturbanceAction::resume);

    presenter.SelectJoint(2);
    view.PressAction(DisturbanceAction::applyTorque);
    view.PressAction(DisturbanceAction::clearTorque);
    view.PressAction(DisturbanceAction::powerOff);

    ASSERT_EQ(4u, posted.size());
    EXPECT_EQ(1u, std::get<arm_host::ResumeJoint>(posted[0]).joint);
    EXPECT_NEAR(30.0f, std::get<arm_host::CollideJoint>(posted[1]).torque.Value(), 1e-6f);
    EXPECT_NEAR(0.0f, std::get<arm_host::CollideJoint>(posted[2]).torque.Value(), 1e-6f);
    EXPECT_EQ(2u, std::get<arm_host::PowerOffMotor>(posted[3]).joint);
}

TEST_F(DisturbancePresenterTest, offers_no_power_off_for_a_motor_that_is_not_up_and_nothing_while_disconnected)
{
    auto snapshot = Virtual();
    snapshot.virtualMotors->motors[0].phase = MotorPhase::stopped;
    presenter.Refresh(snapshot);
    EXPECT_FALSE(presenter.Enabled(DisturbanceAction::powerOff));
    EXPECT_THAT(presenter.Hint(), testing::HasSubstr("Power Off: J1's motor is not up"));
    EXPECT_EQ("Active: —", presenter.Active());

    snapshot.targetReady = false;
    presenter.Refresh(snapshot);
    EXPECT_FALSE(presenter.Enabled(DisturbanceAction::silence));
    EXPECT_THAT(presenter.Hint(), testing::HasSubstr("Silence: not connected"));
}
