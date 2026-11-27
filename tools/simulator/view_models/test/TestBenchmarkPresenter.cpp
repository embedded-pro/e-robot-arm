#include "tools/arm_host/test_doubles/ArmCommandsMock.hpp"
#include "tools/simulator/view_models/BenchmarkPresenter.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <sstream>
#include <vector>

namespace
{
    using namespace testing;
    using namespace std::chrono_literals;
    using ui::theme::StatusLevel;

    class BenchmarkPresenterTest
        : public Test
    {
    public:
        BenchmarkPresenterTest()
        {
            snapshot = simulator::test::Running();
            snapshot.armControllers = { { "gravity-pd", true, true }, { "computed-torque", true, false }, { "slotine-li", false, false }, { "free", true, false, true } };
            for (auto& limit : snapshot.config.positionLimits)
                limit = { foc::Radians{ -2.0f }, foc::Radians{ 2.0f } };
            EXPECT_CALL(arm, Snapshot()).WillRepeatedly(Invoke([this]()
                {
                    return snapshot;
                }));
            Refresh();
        }

        void Refresh(std::span<const arm_host::ArmEvent> events = {}, std::span<const arm_host::TraceSample> trace = {})
        {
            presenter.Refresh(snapshot, events, trace);
        }

        void ExpectPosts()
        {
            EXPECT_CALL(arm, Post(_)).WillRepeatedly(Invoke([this](const arm_host::ArmCommand&)
                {
                    return ++posted;
                }));
        }

        void Accept()
        {
            const std::array events{ arm_host::ArmEvent{ .kind = arm_host::ArmEventKind::accepted, .command = posted } };
            Refresh(events);
        }

        void Event(arm_host::ArmEventKind kind)
        {
            const std::array events{ arm_host::ArmEvent{ .kind = kind } };
            Refresh(events);
        }

        void Trace(float seconds, float error)
        {
            std::vector<arm_host::TraceSample> samples;
            for (float elapsed = 0.0f; elapsed < seconds; elapsed += 0.01f)
            {
                arm_host::TraceSample sample{ .time = time };
                sample.commanded[1] = foc::Radians{ error };
                arm::JointTorques torques{};
                torques[1] = foc::NewtonMeter{ 2.0f };
                sample.torques = torques;
                samples.push_back(sample);
                time += 10ms;
            }
            Refresh({}, samples);
        }

        void RunController(float error)
        {
            Accept();
            Trace(1.05f, 0.0f);
            for (std::size_t leg = 0; leg != 5; ++leg)
            {
                Accept();
                Trace(0.2f, error);
                Event(arm_host::ArmEventKind::moveDone);
            }
            Trace(1.1f, 0.0f);
        }

        void ChooseBoth()
        {
            presenter.Choose(0, true);
            presenter.Choose(1, true);
        }

        StrictMock<arm_host::ArmCommandsMock> arm;
        arm_host::ArmSnapshot snapshot;
        uint32_t posted{ 0 };
        infra::TimePoint time{};
        simulator::BenchmarkPresenter presenter{ arm };
    };
}

TEST_F(BenchmarkPresenterTest, offers_each_enabled_tracking_controller_and_keeps_the_choice_across_refreshes)
{
    ASSERT_EQ(2u, presenter.Choices().size());
    EXPECT_EQ("gravity-pd", presenter.Choices()[0].name);
    EXPECT_EQ(arm::ArmControllerKind::computedTorque, presenter.Choices()[1].kind);
    EXPECT_FALSE(presenter.Choices()[1].chosen);

    presenter.Choose(1, true);
    presenter.Choose(7, true);
    Refresh();
    EXPECT_TRUE(presenter.Choices()[1].chosen);
    EXPECT_FALSE(presenter.Choices()[0].chosen);
}

TEST_F(BenchmarkPresenterTest, run_says_why_it_is_not_offered)
{
    EXPECT_FALSE(presenter.RunState().enabled);
    EXPECT_EQ("Choose at least one controller", presenter.RunState().reason);

    presenter.Choose(0, true);
    snapshot.state = arm::ArmState::ready;
    Refresh();
    EXPECT_EQ("The arm is not enabled", presenter.RunState().reason);

    snapshot.state = arm::ArmState::enabled;
    snapshot.armControllers[0].active = false;
    snapshot.armControllers[3].active = true;
    Refresh();
    EXPECT_EQ("Free mode: select a tracking controller", presenter.RunState().reason);

    snapshot.armControllers[0].active = true;
    snapshot.armControllers[3].active = false;
    Refresh();
    EXPECT_TRUE(presenter.RunState().enabled);
    EXPECT_FALSE(presenter.AbortState().enabled);
    EXPECT_EQ("No benchmark running", presenter.AbortState().reason);
    EXPECT_FALSE(presenter.CanExport());
}

TEST_F(BenchmarkPresenterTest, run_without_a_choice_posts_nothing)
{
    presenter.Run();
    EXPECT_TRUE(presenter.Rows().empty());
    EXPECT_EQ("Choose controllers, then Run", presenter.Status());
}

TEST_F(BenchmarkPresenterTest, running_shows_progress_locks_the_choice_and_offers_abort)
{
    ExpectPosts();
    ChooseBoth();
    presenter.Run();

    EXPECT_EQ("Running 1 of 2: gravity-pd…", presenter.Status());
    EXPECT_FALSE(presenter.RunState().enabled);
    EXPECT_EQ("Running: abort first", presenter.RunState().reason);
    EXPECT_TRUE(presenter.AbortState().enabled);
    ASSERT_EQ(2u, presenter.Rows().size());
    EXPECT_EQ("Running…", presenter.Rows()[0].result);
    EXPECT_EQ("Waiting", presenter.Rows()[1].result);
    EXPECT_EQ("—", presenter.Rows()[0].values[0]);

    presenter.Choose(0, false);
    EXPECT_TRUE(presenter.Choices()[0].chosen);
    EXPECT_FALSE(presenter.CanExport());
}

TEST_F(BenchmarkPresenterTest, a_finished_run_shows_metrics_with_units_and_marks_the_best_per_column)
{
    ExpectPosts();
    ChooseBoth();
    presenter.Run();
    RunController(0.02f);
    EXPECT_EQ("Running 2 of 2: computed-torque…", presenter.Status());
    RunController(0.01f);

    EXPECT_EQ("Done: 2 controllers", presenter.Status());
    EXPECT_EQ(StatusLevel::Ok, presenter.StatusLevel());
    const auto& rows = presenter.Rows();
    ASSERT_EQ(2u, rows.size());
    EXPECT_EQ("Done", rows[0].result);
    EXPECT_EQ(StatusLevel::Ok, rows[0].level);
    EXPECT_THAT(rows[0].values[1], StrEq("0.0200 rad"));
    EXPECT_THAT(rows[1].values[1], StrEq("0.0100 rad ▲"));
    EXPECT_THAT(rows[1].values[0], EndsWith("rad ▲"));
    EXPECT_THAT(rows[0].values[2], EndsWith("N·m ▲"));
    EXPECT_THAT(rows[1].values[2], EndsWith("N·m ▲"));
    EXPECT_THAT(rows[0].values[4], EndsWith(" s ▲"));
    EXPECT_TRUE(presenter.CanExport());
}

TEST_F(BenchmarkPresenterTest, a_single_controller_shows_no_best_marker)
{
    ExpectPosts();
    presenter.Choose(0, true);
    presenter.Run();
    RunController(0.02f);

    EXPECT_EQ("Done: 1 controllers", presenter.Status());
    EXPECT_THAT(presenter.Rows()[0].values[1], StrEq("0.0200 rad"));
}

TEST_F(BenchmarkPresenterTest, the_chart_holds_one_error_series_per_controller_and_advances_its_revision)
{
    ExpectPosts();
    ChooseBoth();
    const auto before = presenter.ChartRevision();
    presenter.Run();
    EXPECT_GT(presenter.ChartRevision(), before);
    EXPECT_TRUE(presenter.Chart().panels.empty());

    RunController(0.02f);
    Accept();
    Trace(1.05f, 0.0f);
    Accept();
    const auto during = presenter.ChartRevision();
    Trace(0.1f, 0.01f);
    EXPECT_GT(presenter.ChartRevision(), during);

    const auto& chart = presenter.Chart();
    ASSERT_EQ(1u, chart.panels.size());
    EXPECT_EQ("Tracking error", chart.panels[0].title);
    ASSERT_EQ(2u, chart.panels[0].series.size());
    EXPECT_EQ("gravity-pd", chart.panels[0].series[0].name);
    EXPECT_EQ(chart.time.size(), chart.panels[0].series[0].data.size());
    EXPECT_THAT(chart.panels[0].series[1].data, AllOf(Not(IsEmpty()), SizeIs(Lt(chart.time.size()))));

    const auto idle = presenter.ChartRevision();
    Refresh();
    EXPECT_EQ(idle, presenter.ChartRevision());
}

TEST_F(BenchmarkPresenterTest, abort_stops_the_run_and_reports_it)
{
    ExpectPosts();
    presenter.Choose(1, true);
    presenter.Run();
    presenter.Abort();

    EXPECT_EQ("Aborted", presenter.Status());
    EXPECT_EQ(StatusLevel::Warning, presenter.StatusLevel());
    EXPECT_EQ("Failed: aborted", presenter.Rows()[0].result);
    EXPECT_EQ(StatusLevel::Fault, presenter.Rows()[0].level);
    EXPECT_TRUE(presenter.RunState().enabled);
    EXPECT_TRUE(presenter.CanExport());

    presenter.Abort();
    EXPECT_EQ("Aborted", presenter.Status());
}

TEST_F(BenchmarkPresenterTest, a_trip_fails_the_benchmark)
{
    ExpectPosts();
    presenter.Choose(1, true);
    presenter.Run();
    Accept();
    const std::array events{ arm_host::ArmEvent{ .kind = arm_host::ArmEventKind::tripped, .reason = "joint 2 fault" } };
    Refresh(events);

    EXPECT_THAT(presenter.Status(), StartsWith("Failed: tripped: joint 2 fault"));
    EXPECT_EQ(StatusLevel::Fault, presenter.StatusLevel());
}

TEST_F(BenchmarkPresenterTest, a_refused_start_is_reported_and_leaves_no_results)
{
    presenter.Choose(1, true);
    Refresh();
    snapshot.moving = true;
    presenter.Run();

    EXPECT_EQ("Not started: the arm is moving", presenter.Status());
    EXPECT_EQ(StatusLevel::Fault, presenter.StatusLevel());
    EXPECT_TRUE(presenter.Rows().empty());
    EXPECT_FALSE(presenter.CanExport());
}

TEST_F(BenchmarkPresenterTest, exports_the_results_as_csv_once_the_run_ends)
{
    ExpectPosts();
    presenter.Choose(0, true);
    std::ostringstream empty;
    presenter.WriteCsv(empty);
    EXPECT_TRUE(empty.str().empty());

    presenter.Run();
    RunController(0.02f);
    std::ostringstream csv;
    presenter.WriteCsv(csv);
    EXPECT_THAT(csv.str(), HasSubstr("gravity-pd"));
}
