#include "tools/arm_host/test_doubles/ArmCommandsMock.hpp"
#include "tools/simulator/view_models/MotorPresenter.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "ui/backend/recording/RecordingFormView.hpp"
#include "gmock/gmock.h"
#include <array>

namespace
{
    using namespace testing;
    using arm::MotorParameter;
    using arm_host::ArmEventKind;
    using simulator::ConfigurationAction;
    using simulator::ConfigurationState;
    using simulator::MotorField;

    MATCHER_P(ReadsMotorOf, joint, "")
    {
        return std::holds_alternative<arm_host::ReadMotor>(arg) && std::get<arm_host::ReadMotor>(arg).joint == static_cast<std::size_t>(joint);
    }

    MATCHER_P(AppliesMotorTo, joint, "")
    {
        return std::holds_alternative<arm_host::ApplyMotor>(arg) && std::get<arm_host::ApplyMotor>(arg).joint == static_cast<std::size_t>(joint);
    }

    class MotorPresenterTest
        : public Test
    {
    public:
        MotorPresenterTest()
        {
            presenter.Refresh(snapshot, {});
        }

        void Answer(const arm_host::ArmEvent& event)
        {
            const std::array events{ event };
            presenter.Refresh(snapshot, events);
        }

        arm_host::ArmEvent ReadEvent(std::size_t joint, const arm::MotorSettings& motor) const
        {
            arm_host::ArmEvent event{ .kind = ArmEventKind::motorRead, .joint = joint };
            event.motor = motor;
            return event;
        }

        ui::backend::recording::RecordingFormView view;
        StrictMock<arm_host::ArmCommandsMock> arm;
        simulator::MotorPresenter presenter{ arm, view };
        arm_host::ArmSnapshot snapshot{ simulator::test::Compatible() };
    };
}

TEST_F(MotorPresenterTest, starts_on_the_first_preset_in_display_units)
{
    EXPECT_EQ(0u, presenter.Preset());
    EXPECT_FALSE(arm::Compare(arm::motorPresets[0].settings, presenter.FormSettings()).Any());
}

TEST_F(MotorPresenterTest, choosing_a_preset_fills_every_field_and_an_edit_turns_it_custom)
{
    view.PickOption(MotorField::preset, 1);
    EXPECT_FALSE(arm::Compare(arm::motorPresets[1].settings, presenter.FormSettings()).Any());

    view.TypeNumber(MotorField::Parameter(MotorParameter::resistance), 0.5);
    EXPECT_EQ(simulator::customPreset, presenter.Preset());
    EXPECT_NEAR(0.5f, presenter.FormSettings()[MotorParameter::resistance], 1.0e-6f);
    EXPECT_NEAR(4.8e-6f, presenter.FormSettings()[MotorParameter::inertia], 1.0e-10f);

    view.TypeNumber(MotorField::Parameter(MotorParameter::resistance), 0.405);
    EXPECT_EQ(1u, presenter.Preset());
}

TEST_F(MotorPresenterTest, read_loads_the_joint_motor_and_names_its_preset_or_how_it_differs)
{
    EXPECT_CALL(arm, Post(ReadsMotorOf(0))).WillOnce(Return(3));
    view.PressAction(ConfigurationAction::read);

    auto motor = arm::motorPresets[1].settings;
    Answer(ReadEvent(0, motor));
    EXPECT_EQ(ConfigurationState::read, presenter.State());
    EXPECT_EQ("Read — Anaheim BLY172S-24V-4000", presenter.StateText());

    view.TypeNumber(MotorField::Parameter(MotorParameter::inertia), 50.0);
    EXPECT_EQ(ConfigurationState::edited, presenter.State());
    EXPECT_EQ("Edited — custom: differs from Anaheim BLY172S-24V-4000 in Rotor inertia", presenter.StateText());
}

TEST_F(MotorPresenterTest, apply_then_verify_lists_every_parameter_the_joint_reports_differently)
{
    EXPECT_CALL(arm, Post(ReadsMotorOf(2))).WillOnce(Return(4));
    presenter.SelectJoint(2);
    Answer(ReadEvent(2, arm::motorPresets[0].settings));

    EXPECT_CALL(arm, Post(AppliesMotorTo(2))).WillOnce(Return(5));
    view.PressAction(ConfigurationAction::apply);
    Answer({ .kind = ArmEventKind::motorApplied, .joint = 2 });
    EXPECT_EQ(ConfigurationState::applied, presenter.State());

    auto reported = arm::motorPresets[0].settings;
    reported[MotorParameter::peakCurrent] = 15.0f;
    EXPECT_CALL(arm, Post(ReadsMotorOf(2))).WillOnce(Return(6));
    view.PressAction(ConfigurationAction::verify);
    Answer(ReadEvent(2, reported));

    EXPECT_EQ(ConfigurationState::differs, presenter.State());
    EXPECT_EQ(ui::theme::StatusLevel::Fault, presenter.StateLevel());
    EXPECT_THAT(presenter.Hint(), HasSubstr("Peak current: applied 20 A, joint reports 15 A"));
}

TEST_F(MotorPresenterTest, a_hidden_panel_reads_its_joint_only_once_it_is_shown)
{
    presenter.SetShown(false);
    presenter.SelectJoint(3);
    EXPECT_EQ(ConfigurationState::unknown, presenter.State());

    EXPECT_CALL(arm, Post(ReadsMotorOf(3))).WillOnce(Return(8));
    presenter.SetShown(true);
    EXPECT_EQ(ConfigurationState::reading, presenter.State());
}

TEST_F(MotorPresenterTest, a_joint_chosen_while_another_is_being_read_is_read_next)
{
    EXPECT_CALL(arm, Post(ReadsMotorOf(0))).WillOnce(Return(8));
    view.PressAction(ConfigurationAction::read);
    presenter.SelectJoint(5);
    EXPECT_EQ(ConfigurationState::unknown, presenter.State());

    EXPECT_CALL(arm, Post(ReadsMotorOf(5))).WillOnce(Return(9));
    Answer(ReadEvent(0, arm::motorPresets[0].settings));
    EXPECT_EQ(ConfigurationState::reading, presenter.State());
}
