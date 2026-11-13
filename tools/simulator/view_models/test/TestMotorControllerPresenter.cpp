#include "tools/arm_host/test_doubles/ArmCommandsMock.hpp"
#include "tools/simulator/view_models/MotorControllerPresenter.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "ui/backend/recording/RecordingFormView.hpp"
#include "gmock/gmock.h"
#include <array>

namespace
{
    using namespace testing;
    using arm_host::ArmEventKind;
    using simulator::ConfigurationState;
    using simulator::MotorControllerAction;
    using simulator::MotorControllerField;
    using ui::backend::recording::FormCommandKind;

    MATCHER_P(Holds, type, "")
    {
        return std::holds_alternative<std::remove_cvref_t<decltype(type)>>(arg);
    }

    MATCHER_P(ReadsJoint, joint, "")
    {
        return std::holds_alternative<arm_host::ReadController>(arg) && std::get<arm_host::ReadController>(arg).joint == static_cast<std::size_t>(joint);
    }

    MATCHER_P(AppliesJoint, joint, "")
    {
        return std::holds_alternative<arm_host::ApplyController>(arg) && std::get<arm_host::ApplyController>(arg).joint == static_cast<std::size_t>(joint);
    }

    class MotorControllerPresenterTest
        : public Test
    {
    public:
        bool ActionEnabled(ui::model::ActionId action) const
        {
            bool enabled = true;
            for (const auto& command : view.Commands())
                if (command.kind == FormCommandKind::SetActionEnabled && command.action == action)
                    enabled = command.flag;
            return enabled;
        }

        void Answer(const arm_host::ArmEvent& event)
        {
            const std::array events{ event };
            presenter.Refresh(snapshot, events);
        }

        arm_host::ArmEvent ReadEvent(std::size_t joint, const arm::ControllerSettings& settings) const
        {
            arm_host::ArmEvent event{ .kind = ArmEventKind::controllerRead, .joint = joint };
            event.controller = { can::FocMotorMode::position, settings };
            return event;
        }

        ui::backend::recording::RecordingFormView view;
        StrictMock<arm_host::ArmCommandsMock> arm;
        simulator::MotorControllerPresenter presenter{ arm, view };
        arm_host::ArmSnapshot snapshot{ simulator::test::Compatible() };
    };
}

TEST_F(MotorControllerPresenterTest, builds_the_joint_the_three_algorithms_and_the_fourteen_gains)
{
    EXPECT_EQ(18u, static_cast<std::size_t>(std::ranges::count_if(view.Commands(), [](const auto& command)
                       {
                           return command.kind == FormCommandKind::CreateNumber || command.kind == FormCommandKind::CreateChoice || command.kind == FormCommandKind::CreateToggle;
                       })));
    EXPECT_EQ(4u, static_cast<std::size_t>(std::ranges::count(view.Commands(), FormCommandKind::CreateAction, &ui::backend::recording::FormCommand::kind)));
}

TEST_F(MotorControllerPresenterTest, shows_only_the_gains_the_selected_algorithms_use)
{
    EXPECT_TRUE(view.IsControlVisible(MotorControllerField::Gain(arm::Gain::speedBandwidth)));
    EXPECT_FALSE(view.IsControlVisible(MotorControllerField::Gain(arm::Gain::speedErrorWeight)));
    EXPECT_FALSE(view.IsControlVisible(MotorControllerField::Gain(arm::Gain::currentSwitchingGain)));

    view.PickOption(MotorControllerField::speed, static_cast<std::size_t>(can::FocSpeedAlgorithm::lqi));
    view.PickOption(MotorControllerField::current, static_cast<std::size_t>(can::FocCurrentAlgorithm::slidingMode));

    EXPECT_TRUE(view.IsControlVisible(MotorControllerField::Gain(arm::Gain::speedErrorWeight)));
    EXPECT_TRUE(view.IsControlVisible(MotorControllerField::Gain(arm::Gain::speedIntegralWeight)));
    EXPECT_FALSE(view.IsControlVisible(MotorControllerField::Gain(arm::Gain::speedObserverBandwidthRatio)));
    EXPECT_TRUE(view.IsControlVisible(MotorControllerField::Gain(arm::Gain::currentBoundaryLayer)));
}

TEST_F(MotorControllerPresenterTest, actions_need_a_ready_arm_and_say_why_otherwise)
{
    auto enabled = simulator::test::Running();
    presenter.Refresh(enabled, {});

    EXPECT_FALSE(ActionEnabled(MotorControllerAction::apply));
    EXPECT_THAT(presenter.Hint(), HasSubstr("Not ready: enabled"));

    presenter.Refresh(snapshot, {});
    EXPECT_TRUE(ActionEnabled(MotorControllerAction::apply));
    EXPECT_TRUE(ActionEnabled(MotorControllerAction::read));
    EXPECT_FALSE(ActionEnabled(MotorControllerAction::verify));
    EXPECT_THAT(presenter.Hint(), HasSubstr("Verify: Nothing applied yet"));
}

TEST_F(MotorControllerPresenterTest, read_loads_the_joint_settings_into_the_form)
{
    presenter.Refresh(snapshot, {});
    EXPECT_CALL(arm, Post(ReadsJoint(0))).WillOnce(Return(5));
    view.PressAction(MotorControllerAction::read);
    EXPECT_EQ(ConfigurationState::reading, presenter.State());
    EXPECT_FALSE(ActionEnabled(MotorControllerAction::apply));

    arm::ControllerSettings reported{};
    reported.speed = can::FocSpeedAlgorithm::adrc;
    reported[arm::Gain::speedObserverBandwidthRatio] = 7.5f;
    Answer(ReadEvent(0, reported));

    EXPECT_EQ(ConfigurationState::read, presenter.State());
    EXPECT_EQ("Read — mode position (reported)", presenter.StateText());
    EXPECT_FALSE(arm::Compare(reported, presenter.FormSettings()).Any());
    EXPECT_TRUE(view.IsControlVisible(MotorControllerField::Gain(arm::Gain::speedObserverBandwidthRatio)));
}

TEST_F(MotorControllerPresenterTest, an_edit_after_reading_marks_the_form_edited)
{
    presenter.Refresh(snapshot, {});
    EXPECT_CALL(arm, Post(ReadsJoint(0))).WillOnce(Return(5));
    view.PressAction(MotorControllerAction::read);
    Answer(ReadEvent(0, {}));

    view.TypeNumber(MotorControllerField::Gain(arm::Gain::speedBandwidth), 150.0);

    EXPECT_EQ(ConfigurationState::edited, presenter.State());
    EXPECT_EQ(ui::theme::StatusLevel::Warning, presenter.StateLevel());
}

TEST_F(MotorControllerPresenterTest, apply_then_verify_reports_verified_when_the_joint_matches)
{
    presenter.Refresh(snapshot, {});
    view.TypeNumber(MotorControllerField::Gain(arm::Gain::speedBandwidth), 150.0);
    EXPECT_CALL(arm, Post(AppliesJoint(0))).WillOnce(Invoke([](const arm_host::ArmCommand& command)
        {
            EXPECT_NEAR(150.0f, std::get<arm_host::ApplyController>(command).settings[arm::Gain::speedBandwidth], 1.0e-4f);
            return 6u;
        }));
    view.PressAction(MotorControllerAction::apply);
    Answer({ .kind = ArmEventKind::controllerApplied, .joint = 0 });
    EXPECT_EQ(ConfigurationState::applied, presenter.State());
    EXPECT_TRUE(ActionEnabled(MotorControllerAction::verify));

    EXPECT_CALL(arm, Post(ReadsJoint(0))).WillOnce(Return(7));
    view.PressAction(MotorControllerAction::verify);
    Answer(ReadEvent(0, presenter.FormSettings()));

    EXPECT_EQ(ConfigurationState::verified, presenter.State());
    EXPECT_EQ(ui::theme::StatusLevel::Ok, presenter.StateLevel());
}

TEST_F(MotorControllerPresenterTest, verify_lists_every_item_the_joint_reports_differently)
{
    presenter.Refresh(snapshot, {});
    view.TypeNumber(MotorControllerField::Gain(arm::Gain::speedBandwidth), 150.0);
    EXPECT_CALL(arm, Post(AppliesJoint(0))).WillOnce(Return(6));
    view.PressAction(MotorControllerAction::apply);
    Answer({ .kind = ArmEventKind::controllerApplied, .joint = 0 });

    auto reported = presenter.FormSettings();
    reported[arm::Gain::speedBandwidth] = 120.0f;
    reported.position = can::FocPositionAlgorithm::lqr;
    EXPECT_CALL(arm, Post(ReadsJoint(0))).WillOnce(Return(7));
    view.PressAction(MotorControllerAction::verify);
    Answer(ReadEvent(0, reported));

    EXPECT_EQ(ConfigurationState::differs, presenter.State());
    EXPECT_THAT(presenter.Hint(), HasSubstr("Speed bandwidth: applied 150, joint reports 120"));
    EXPECT_THAT(presenter.Hint(), HasSubstr("Position algorithm: applied PID, joint reports LQR"));
}

TEST_F(MotorControllerPresenterTest, apply_to_all_joints_applies_them_one_after_the_other)
{
    presenter.Refresh(snapshot, {});
    InSequence order;
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_CALL(arm, Post(AppliesJoint(joint))).WillOnce(Return(static_cast<uint32_t>(10 + joint)));

    view.PressAction(MotorControllerAction::applyToAll);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_EQ(ConfigurationState::applying, presenter.State());
        Answer({ .kind = ArmEventKind::controllerApplied, .joint = joint });
    }

    EXPECT_EQ(ConfigurationState::applied, presenter.State());
}

TEST_F(MotorControllerPresenterTest, a_refused_or_failed_operation_shows_the_reason)
{
    presenter.Refresh(snapshot, {});
    EXPECT_CALL(arm, Post(AppliesJoint(0))).WillOnce(Return(6));
    view.PressAction(MotorControllerAction::apply);
    Answer({ .kind = ArmEventKind::refused, .command = 6, .reason = "busy" });
    EXPECT_EQ(ConfigurationState::failed, presenter.State());
    EXPECT_THAT(presenter.Hint(), HasSubstr("busy"));

    EXPECT_CALL(arm, Post(ReadsJoint(0))).WillOnce(Return(8));
    view.PressAction(MotorControllerAction::read);
    Answer({ .kind = ArmEventKind::controllerFailed, .reason = "no answer within 100 ms", .joint = 0 });
    EXPECT_THAT(presenter.Hint(), HasSubstr("no answer within 100 ms"));
    EXPECT_EQ(ui::theme::StatusLevel::Fault, presenter.StateLevel());
}

TEST_F(MotorControllerPresenterTest, choosing_another_joint_reads_it_and_tells_the_window)
{
    presenter.Refresh(snapshot, {});
    std::size_t selected = 99;
    presenter.SetJointSelected([&selected](std::size_t joint)
        {
            selected = joint;
        });

    EXPECT_CALL(arm, Post(ReadsJoint(4))).WillOnce(Return(3));
    view.PickOption(MotorControllerField::joint, 4);

    EXPECT_EQ(4u, presenter.Joint());
    EXPECT_EQ(4u, selected);
    EXPECT_EQ(ConfigurationState::reading, presenter.State());
}

TEST_F(MotorControllerPresenterTest, a_selection_made_elsewhere_selects_the_joint_here)
{
    presenter.Refresh(simulator::test::Running(), {});

    presenter.SelectJoint(2);

    EXPECT_EQ(2u, presenter.Joint());
    EXPECT_EQ(ConfigurationState::unknown, presenter.State());
}
