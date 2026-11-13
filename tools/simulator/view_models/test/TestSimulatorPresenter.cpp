#include "tools/arm_host/test_doubles/ArmCommandsMock.hpp"
#include "tools/simulator/view_models/SimulatorPresenter.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "ui/backend/recording/RecordingFormView.hpp"
#include "ui/backend/recording/RecordingShell.hpp"
#include "gmock/gmock.h"
#include <algorithm>

namespace
{
    using namespace testing;
    using arm_host::ArmEventKind;
    using simulator::MoveAction;
    using simulator::OperateAction;
    using simulator::SimulatorPresenter;
    using ui::backend::recording::FormCommandKind;

    MATCHER_P(Holds, type, "")
    {
        return std::holds_alternative<std::remove_cvref_t<decltype(type)>>(arg);
    }

    class SimulatorPresenterTest
        : public Test
    {
    public:
        bool ActionEnabled(const ui::backend::recording::RecordingFormView& view, ui::model::ActionId action) const
        {
            bool enabled = true;
            for (const auto& command : view.Commands())
                if (command.kind == FormCommandKind::SetActionEnabled && command.action == action)
                    enabled = command.flag;
            return enabled;
        }

        std::size_t Count(const ui::backend::recording::RecordingFormView& view, FormCommandKind kind) const
        {
            return static_cast<std::size_t>(std::ranges::count(view.Commands(), kind, &ui::backend::recording::FormCommand::kind));
        }

        void Trigger(ui::model::ActionId action)
        {
            operateView.PressAction(action);
        }

        ui::shell::ShellSpec spec{ "e-robot-arm Simulator", {}, 350.0f, {}, {} };
        ui::backend::recording::RecordingShell shell{ spec };
        ui::backend::recording::RecordingFormView operateView;
        ui::backend::recording::RecordingFormView moveView;
        StrictMock<arm_host::ArmCommandsMock> arm;
        SimulatorPresenter presenter{ arm, shell, operateView, moveView, simulator::test::Limits(), infra::TimePoint{} };
    };
}

TEST_F(SimulatorPresenterTest, builds_the_operate_and_move_forms_from_their_specs)
{
    EXPECT_EQ(5u, Count(operateView, FormCommandKind::CreateAction));
    EXPECT_EQ(6u, Count(moveView, FormCommandKind::CreateNumber));
    EXPECT_EQ(2u, Count(moveView, FormCommandKind::CreateAction));
}

TEST_F(SimulatorPresenterTest, without_a_target_disables_the_operations_and_says_why)
{
    EXPECT_FALSE(ActionEnabled(operateView, OperateAction::bringUp));
    EXPECT_FALSE(ActionEnabled(moveView, MoveAction::move));
    EXPECT_THAT(presenter.OperateHint(), HasSubstr("Bring Up: Not connected"));
    EXPECT_THAT(presenter.MoveHint(), HasSubstr("Move: Not connected"));
    EXPECT_EQ("Disconnected", presenter.Status().connection);
}

TEST_F(SimulatorPresenterTest, enables_each_lifecycle_step_as_the_arm_allows_it)
{
    presenter.Refresh(simulator::test::Connected(), {});
    EXPECT_TRUE(ActionEnabled(operateView, OperateAction::bringUp));
    EXPECT_FALSE(ActionEnabled(operateView, OperateAction::enable));
    EXPECT_THAT(presenter.OperateHint(), HasSubstr("Enable: Bring the arm up first"));

    presenter.Refresh(simulator::test::Compatible(), {});
    EXPECT_FALSE(ActionEnabled(operateView, OperateAction::bringUp));
    EXPECT_TRUE(ActionEnabled(operateView, OperateAction::enable));
    EXPECT_FALSE(ActionEnabled(operateView, OperateAction::disable));
    EXPECT_EQ("Contracts: compatible", presenter.Status().contracts);

    presenter.Refresh(simulator::test::Running(), {});
    EXPECT_FALSE(ActionEnabled(operateView, OperateAction::enable));
    EXPECT_TRUE(ActionEnabled(operateView, OperateAction::disable));
    EXPECT_TRUE(ActionEnabled(moveView, MoveAction::move));
}

TEST_F(SimulatorPresenterTest, an_operation_posts_its_command_and_shows_it_requested_until_answered)
{
    EXPECT_CALL(arm, Post(Holds(arm_host::Enable{}))).WillOnce(Return(42));
    Trigger(OperateAction::enable);
    EXPECT_EQ("Enable requested…", shell.Statuses().back());

    const std::array events{ arm_host::ArmEvent{ .kind = ArmEventKind::refused, .command = 42, .commandName = "enable", .reason = "tripped" } };
    presenter.Refresh(simulator::test::Compatible(), events);

    EXPECT_EQ("Enable refused: tripped", shell.Statuses().back());
    EXPECT_EQ(1u, presenter.Log().Lines().size());
}

TEST_F(SimulatorPresenterTest, each_operate_action_posts_its_command)
{
    InSequence order;
    EXPECT_CALL(arm, Post(Holds(arm_host::BringUp{}))).WillOnce(Return(1));
    EXPECT_CALL(arm, Post(Holds(arm_host::Enable{}))).WillOnce(Return(2));
    EXPECT_CALL(arm, Post(Holds(arm_host::Disable{}))).WillOnce(Return(3));
    EXPECT_CALL(arm, Post(Holds(arm_host::ClearFaults{}))).WillOnce(Return(4));

    Trigger(OperateAction::bringUp);
    Trigger(OperateAction::enable);
    Trigger(OperateAction::disable);
    Trigger(OperateAction::clearFaults);
}

TEST_F(SimulatorPresenterTest, a_single_step_posts_its_command_only_while_its_rule_allows_it)
{
    EXPECT_FALSE(presenter.SingleStep(simulator::Control::start));

    presenter.Refresh(simulator::test::Compatible(), {});
    InSequence order;
    EXPECT_CALL(arm, Post(Holds(arm_host::CheckContracts{}))).WillOnce(Return(1));
    EXPECT_CALL(arm, Post(Holds(arm_host::SelectMode{}))).WillOnce(Return(2));
    EXPECT_CALL(arm, Post(Holds(arm_host::StartJoints{}))).WillOnce(Return(3));
    EXPECT_TRUE(presenter.SingleStep(simulator::Control::checkContracts));
    EXPECT_TRUE(presenter.SingleStep(simulator::Control::positionMode));
    EXPECT_TRUE(presenter.SingleStep(simulator::Control::start));
    EXPECT_FALSE(presenter.SingleStep(simulator::Control::stop));
    EXPECT_FALSE(presenter.SingleStep(simulator::Control::move));
}

TEST_F(SimulatorPresenterTest, move_posts_the_six_targets_from_the_form)
{
    presenter.Refresh(simulator::test::Running(), {});
    moveView.TypeNumber(ui::model::FieldId{ 2 }, 1.25);

    EXPECT_CALL(arm, Post(Holds(arm_host::MoveTo{})))
        .WillOnce(Invoke([](const arm_host::ArmCommand& command)
            {
                EXPECT_NEAR(1.25f, std::get<arm_host::MoveTo>(command).target[2].Value(), 1.0e-5f);
                return 7u;
            }));
    moveView.PressAction(MoveAction::move);
}

TEST_F(SimulatorPresenterTest, move_is_not_posted_while_the_rules_forbid_it)
{
    presenter.Refresh(simulator::test::Compatible(), {});

    moveView.PressAction(MoveAction::move);
}

TEST_F(SimulatorPresenterTest, the_emergency_stop_posts_whatever_the_state)
{
    EXPECT_CALL(arm, Post(Holds(arm_host::EmergencyStop{}))).WillOnce(Return(1));

    presenter.EmergencyStop();
}

TEST_F(SimulatorPresenterTest, a_trip_names_its_source_in_the_status_and_in_the_rearm_prompt)
{
    auto tripped = simulator::test::Running();
    tripped.tripped = true;
    tripped.state = arm::ArmState::tripped;
    const std::array events{ arm_host::ArmEvent{ .kind = ArmEventKind::tripped, .joint = 2, .fault = can::FocFaultCode::overCurrent } };
    presenter.Refresh(tripped, events);

    std::string prompt;
    presenter.SetRearmPrompt([&prompt](std::string_view text)
        {
            prompt = text;
        });
    Trigger(OperateAction::rearm);

    EXPECT_EQ("Tripped — Joint 3 (Overcurrent)", presenter.Status().trip);
    EXPECT_EQ(ui::theme::StatusLevel::Fault, presenter.Status().tripLevel);
    EXPECT_EQ("Re-arm after the trip by Joint 3 (Overcurrent)?", prompt);
}

TEST_F(SimulatorPresenterTest, confirming_the_rearm_posts_it)
{
    EXPECT_CALL(arm, Post(Holds(arm_host::Rearm{}))).WillOnce(Return(1));

    presenter.ConfirmRearm();
}

TEST_F(SimulatorPresenterTest, injects_failures_only_on_simulated_joints)
{
    auto real = simulator::test::Running();
    real.target = { arm_host::TargetKind::real, "socketcan:can0" };
    presenter.Refresh(real, {});
    EXPECT_FALSE(presenter.InjectFault(1));
    EXPECT_FALSE(presenter.SilenceJoint(1));
    EXPECT_TRUE(presenter.Status().realArm);
    EXPECT_EQ("REAL ARM — socketcan:can0", presenter.Status().target);

    presenter.Refresh(simulator::test::Running(), {});
    EXPECT_CALL(arm, Post(Holds(arm_host::InjectFault{}))).WillOnce(Return(1));
    EXPECT_CALL(arm, Post(Holds(arm_host::SilenceJoint{}))).WillOnce(Return(2));
    EXPECT_TRUE(presenter.InjectFault(1));
    EXPECT_TRUE(presenter.SilenceJoint(1));
    EXPECT_FALSE(presenter.InjectFault(6));
}

TEST_F(SimulatorPresenterTest, a_picked_joint_becomes_the_selection)
{
    EXPECT_TRUE(presenter.SelectJoint(4));
    EXPECT_EQ(4u, presenter.Scene().Selected());
}

TEST_F(SimulatorPresenterTest, shows_the_lifecycle_state_with_its_level)
{
    EXPECT_EQ("Disconnected", presenter.Status().stage);

    presenter.Refresh(simulator::test::Connected(), {});
    EXPECT_EQ("Unpowered", presenter.Status().stage);

    presenter.Refresh(simulator::test::Compatible(), {});
    EXPECT_EQ("Ready", presenter.Status().stage);
    EXPECT_EQ(ui::theme::StatusLevel::Neutral, presenter.Status().stageLevel);

    presenter.Refresh(simulator::test::Running(), {});
    EXPECT_EQ("Enabled", presenter.Status().stage);
    EXPECT_EQ(ui::theme::StatusLevel::Ok, presenter.Status().stageLevel);

    auto tripped = simulator::test::Running();
    tripped.state = arm::ArmState::tripped;
    presenter.Refresh(tripped, {});
    EXPECT_EQ("Tripped", presenter.Status().stage);
    EXPECT_EQ(ui::theme::StatusLevel::Fault, presenter.Status().stageLevel);
}

TEST_F(SimulatorPresenterTest, waiting_for_calibration_names_every_joint_not_yet_ready)
{
    auto waiting = simulator::test::Compatible();
    waiting.state = arm::ArmState::waitingForCalibration;
    for (auto& reading : waiting.readings)
        reading.state = can::FocMotorState::ready;
    waiting.readings[1].state = can::FocMotorState::idle;
    waiting.readings[4].state = can::FocMotorState::partialCalibration;

    presenter.Refresh(waiting, {});

    EXPECT_EQ("Waiting for calibration: J2, J5", presenter.Status().stage);
    EXPECT_EQ(ui::theme::StatusLevel::Warning, presenter.Status().stageLevel);
}

TEST_F(SimulatorPresenterTest, a_snapshot_that_stops_changing_is_shown_as_no_data)
{
    auto snapshot = simulator::test::Running();
    snapshot.sequence = 5;
    for (std::size_t frame = 0; frame != SimulatorPresenter::staleFrames; ++frame)
        presenter.Refresh(snapshot, {});
    EXPECT_EQ("Connected", presenter.Status().connection);

    presenter.Refresh(snapshot, {});
    EXPECT_EQ("No data from the arm", presenter.Status().connection);
    EXPECT_EQ(ui::theme::StatusLevel::Warning, presenter.Status().connectionLevel);

    snapshot.sequence = 6;
    presenter.Refresh(snapshot, {});
    EXPECT_EQ("Connected", presenter.Status().connection);
}

TEST_F(SimulatorPresenterTest, a_target_switch_shows_requested_then_its_outcome)
{
    const arm_host::ArmTarget realArm{ arm_host::TargetKind::real, "socketcan:can0" };
    EXPECT_CALL(arm, SwitchTarget(realArm)).WillOnce(Return(12));
    presenter.SwitchTarget(realArm);
    EXPECT_EQ("Switch to REAL ARM — socketcan:can0 requested…", shell.Statuses().back());

    const std::array events{ arm_host::ArmEvent{ .kind = ArmEventKind::targetFailed, .command = 12, .commandName = "switch target", .reason = "the adapter could not be opened" } };
    presenter.Refresh(simulator::test::Running(), events);

    EXPECT_EQ("Switch to REAL ARM — socketcan:can0 failed: the adapter could not be opened", shell.Statuses().back());
}

TEST_F(SimulatorPresenterTest, a_rearm_prompt_without_a_known_source_says_so)
{
    std::string prompt;
    presenter.SetRearmPrompt([&prompt](std::string_view text)
        {
            prompt = text;
        });
    Trigger(OperateAction::rearm);

    EXPECT_EQ("Re-arm after the trip by an unknown source?", prompt);
}
