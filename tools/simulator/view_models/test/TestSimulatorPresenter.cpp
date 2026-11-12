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
    EXPECT_EQ(6u, Count(operateView, FormCommandKind::CreateAction));
    EXPECT_EQ(6u, Count(moveView, FormCommandKind::CreateNumber));
    EXPECT_EQ(2u, Count(moveView, FormCommandKind::CreateAction));
}

TEST_F(SimulatorPresenterTest, without_a_target_disables_the_operations_and_says_why)
{
    EXPECT_FALSE(ActionEnabled(operateView, OperateAction::start));
    EXPECT_FALSE(ActionEnabled(moveView, MoveAction::move));
    EXPECT_THAT(presenter.OperateHint(), HasSubstr("Start: Not connected"));
    EXPECT_THAT(presenter.MoveHint(), HasSubstr("Move: Not connected"));
    EXPECT_EQ("Disconnected", presenter.Status().connection);
}

TEST_F(SimulatorPresenterTest, enables_the_bring_up_steps_as_the_arm_allows_them)
{
    presenter.Refresh(simulator::test::Compatible(), {});

    EXPECT_TRUE(ActionEnabled(operateView, OperateAction::positionMode));
    EXPECT_TRUE(ActionEnabled(operateView, OperateAction::start));
    EXPECT_FALSE(ActionEnabled(operateView, OperateAction::stop));
    EXPECT_EQ("Contracts: compatible", presenter.Status().contracts);
}

TEST_F(SimulatorPresenterTest, an_operation_posts_its_command_and_shows_it_requested_until_answered)
{
    EXPECT_CALL(arm, Post(Holds(arm_host::StartJoints{}))).WillOnce(Return(42));
    Trigger(OperateAction::start);
    EXPECT_EQ("Start requested…", shell.Statuses().back());

    const std::array events{ arm_host::ArmEvent{ .kind = ArmEventKind::refused, .command = 42, .commandName = "start", .reason = "tripped" } };
    presenter.Refresh(simulator::test::Compatible(), events);

    EXPECT_EQ("Start refused: tripped", shell.Statuses().back());
    EXPECT_EQ(1u, presenter.Log().Lines().size());
}

TEST_F(SimulatorPresenterTest, each_operate_action_posts_its_command)
{
    InSequence order;
    EXPECT_CALL(arm, Post(Holds(arm_host::CheckContracts{}))).WillOnce(Return(1));
    EXPECT_CALL(arm, Post(Holds(arm_host::SelectMode{}))).WillOnce(Return(2));
    EXPECT_CALL(arm, Post(Holds(arm_host::StopJoints{}))).WillOnce(Return(3));
    EXPECT_CALL(arm, Post(Holds(arm_host::ClearFaults{}))).WillOnce(Return(4));

    Trigger(OperateAction::checkContracts);
    Trigger(OperateAction::positionMode);
    Trigger(OperateAction::stop);
    Trigger(OperateAction::clearFaults);
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
