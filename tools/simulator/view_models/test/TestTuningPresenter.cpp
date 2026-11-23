#include "tools/arm_host/test_doubles/ArmCommandsMock.hpp"
#include "tools/simulator/view_models/TuningPresenter.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "ui/backend/recording/RecordingFormView.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <sstream>
#include <vector>

namespace
{
    using namespace testing;
    using simulator::TuningAction;
    using simulator::TuningField;

    class TuningPresenterTest
        : public Test
    {
    public:
        TuningPresenterTest()
        {
            EXPECT_CALL(arm, Snapshot()).WillRepeatedly(Invoke([this]()
                {
                    return snapshot;
                }));
            snapshot = simulator::test::Running();
            snapshot.commanded = arm::JointPositions{};
            snapshot.armControllers = { { "gravity-pd", true, true }, { "gravity-pid", true, false } };
        }

        void Refresh(std::span<const arm_host::ArmEvent> events = {}, std::span<const arm_host::TraceSample> trace = {})
        {
            presenter.Refresh(snapshot, events, trace);
        }

        void Accept(uint32_t command)
        {
            const std::array events{ arm_host::ArmEvent{ .kind = arm_host::ArmEventKind::accepted, .command = command } };
            Refresh(events);
        }

        void Trace(std::size_t count, float measured, uint64_t droppedBeforeFirst = 0)
        {
            std::vector<arm_host::TraceSample> samples;
            for (std::size_t index = 0; index != count; ++index)
            {
                arm_host::TraceSample sample{ .time = time, .droppedBefore = index == 0 ? droppedBeforeFirst : 0 };
                sample.commanded[2] = foc::Radians{ 0.05f };
                sample.measured[2] = foc::Radians{ measured };
                samples.push_back(sample);
                time += std::chrono::milliseconds{ 10 };
            }
            Refresh({}, samples);
        }

        void RunStep(uint32_t command, float measured)
        {
            EXPECT_CALL(arm, Post(_)).WillOnce(Return(command));
            view.PressAction(TuningAction::run);
            Accept(command);
            Trace(201, measured);
        }

        StrictMock<arm_host::ArmCommandsMock> arm;
        arm_host::ArmSnapshot snapshot;
        infra::TimePoint time{};
        ui::backend::recording::RecordingFormView view;
        simulator::TuningPresenter presenter{ view, arm };
    };
}

TEST_F(TuningPresenterTest, run_is_offered_only_on_an_enabled_idle_arm_and_says_why_not)
{
    snapshot.state = arm::ArmState::ready;
    Refresh();
    EXPECT_FALSE(presenter.Enabled(TuningAction::run));
    EXPECT_THAT(presenter.Hint(), HasSubstr("Run: the arm is not enabled"));

    snapshot.state = arm::ArmState::enabled;
    Refresh();
    EXPECT_TRUE(presenter.Enabled(TuningAction::run));

    view.TypeNumber(TuningField::amplitude, 0.3);
    Refresh();
    EXPECT_FALSE(presenter.Enabled(TuningAction::run));
    EXPECT_THAT(presenter.Hint(), HasSubstr("Run: a step is not 0 and at most 0.2 rad"));
    view.PressAction(TuningAction::run);
}

TEST_F(TuningPresenterTest, run_steps_the_selected_joint_shows_its_progress_then_its_metrics)
{
    presenter.SelectJoint(2);
    Refresh();
    EXPECT_CALL(arm, Post(_)).WillOnce(Invoke([](const arm_host::ArmCommand& command)
        {
            EXPECT_NEAR(0.05f, std::get<arm_host::StepTo>(command).target[2].Value(), 1.0e-6f);
            return 5u;
        }));
    view.PressAction(TuningAction::run);
    EXPECT_EQ("Requested step on J3…", presenter.Status());
    EXPECT_FALSE(presenter.Enabled(TuningAction::run));

    Accept(5);
    EXPECT_EQ("Running step on J3…", presenter.Status());
    Trace(201, 0.05f);

    EXPECT_EQ("Done: gravity-pd · step 0.050 rad · J3", presenter.Status());
    EXPECT_EQ(ui::theme::StatusLevel::Ok, presenter.StatusLevel());
    EXPECT_EQ("Rise time", presenter.Metrics()[0].name);
    EXPECT_EQ("0.000 s", presenter.Metrics()[0].current);
    EXPECT_EQ("—", presenter.Metrics()[0].reference);
    EXPECT_EQ("0.0000 rad", presenter.Metrics()[4].current);
    EXPECT_TRUE(presenter.CanExport());
    std::ostringstream csv;
    presenter.WriteCsv(csv);
    EXPECT_THAT(csv.str(), StartsWith("time_s,commanded_rad,measured_rad,tracking_error_rad,torque_nm,dropped_before\n0,0.05,0.05,0,0,0\n"));
    ASSERT_EQ(3u, presenter.Chart().panels.size());
    EXPECT_EQ(201u, presenter.Chart().time.size());
}

TEST_F(TuningPresenterTest, a_kept_reference_is_overlaid_and_compared_with_the_next_run)
{
    presenter.SelectJoint(2);
    Refresh();
    RunStep(5, 0.05f);
    EXPECT_TRUE(presenter.Enabled(TuningAction::keepReference));
    EXPECT_FALSE(presenter.Enabled(TuningAction::clearReference));

    view.PressAction(TuningAction::keepReference);
    EXPECT_FALSE(presenter.Enabled(TuningAction::keepReference));
    EXPECT_TRUE(presenter.Enabled(TuningAction::clearReference));
    EXPECT_EQ(3u, presenter.Chart().panels[0].series.size());

    RunStep(6, 0.04f);
    EXPECT_EQ("0.0000 rad", presenter.Metrics()[4].reference);
    EXPECT_EQ("0.0100 rad", presenter.Metrics()[4].current);
    EXPECT_EQ(3u, presenter.Chart().panels[0].series.size());
    EXPECT_EQ("Ref meas", presenter.Chart().panels[0].series[2].name);

    view.PressAction(TuningAction::clearReference);
    EXPECT_EQ("—", presenter.Metrics()[4].reference);
    EXPECT_EQ(2u, presenter.Chart().panels[0].series.size());
}

TEST_F(TuningPresenterTest, a_run_that_fails_says_why_and_offers_nothing_to_export)
{
    Refresh();
    EXPECT_CALL(arm, Post(_)).WillOnce(Return(5u));
    view.PressAction(TuningAction::run);
    Accept(5);
    const std::array events{ arm_host::ArmEvent{ .kind = arm_host::ArmEventKind::tripped, .reason = "collision 25.0 N·m" } };
    Refresh(events);

    EXPECT_EQ("Failed: tripped: collision 25.0 N·m", presenter.Status());
    EXPECT_EQ(ui::theme::StatusLevel::Fault, presenter.StatusLevel());
    EXPECT_FALSE(presenter.CanExport());
    EXPECT_TRUE(presenter.Enabled(TuningAction::run));
}

TEST_F(TuningPresenterTest, a_target_switch_drops_the_runs_and_the_reference)
{
    Refresh();
    RunStep(5, 0.05f);
    view.PressAction(TuningAction::keepReference);

    snapshot.target = { arm_host::TargetKind::virtualArm, "virtual:arm", "e-foc-virtual-motor" };
    Refresh();

    EXPECT_FALSE(presenter.CanExport());
    EXPECT_FALSE(presenter.Enabled(TuningAction::clearReference));
    EXPECT_EQ("", presenter.Status());
    EXPECT_TRUE(presenter.Chart().panels.empty());
}

TEST_F(TuningPresenterTest, a_run_that_lost_samples_says_so_as_a_warning)
{
    presenter.SelectJoint(2);
    Refresh();
    EXPECT_CALL(arm, Post(_)).WillOnce(Return(5u));
    view.PressAction(TuningAction::run);
    Accept(5);
    Trace(100, 0.05f);
    Trace(101, 0.05f, 12);

    EXPECT_EQ("Done: gravity-pd · step 0.050 rad · J3 — 12 samples lost during the run; the metrics may be off", presenter.Status());
    EXPECT_EQ(ui::theme::StatusLevel::Warning, presenter.StatusLevel());
}
