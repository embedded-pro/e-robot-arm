#include "tools/simulator/view_models/EventLog.hpp"
#include "gmock/gmock.h"
#include <array>
#include <chrono>

namespace
{
    using namespace std::chrono_literals;
    using arm_host::ArmEventKind;

    class EventLogTest
        : public testing::Test
    {
    public:
        arm_host::ArmEvent At(arm_host::ArmEvent event, infra::Duration after) const
        {
            event.time = start + after;
            return event;
        }

        infra::TimePoint start{};
        simulator::EventLog log{ start, 3 };
    };
}

TEST_F(EventLogTest, describes_events_in_plain_language_with_joints_from_one)
{
    EXPECT_EQ("Joint 3 fault: Stall", simulator::Describe({ .kind = ArmEventKind::jointFault, .joint = 2, .fault = can::FocFaultCode::stall }, start).message);
    EXPECT_EQ("Tripped by Joint 6", simulator::Describe({ .kind = ArmEventKind::tripped, .joint = 5 }, start).message);
    EXPECT_EQ("Start refused: tripped", simulator::Describe({ .kind = ArmEventKind::refused, .commandName = "start", .reason = "tripped" }, start).message);
    EXPECT_EQ(simulator::LogLevel::fault, simulator::Describe({ .kind = ArmEventKind::jointSilent, .joint = 0 }, start).level);
    const auto tracking = simulator::Describe({ .kind = ArmEventKind::supervisionTrip, .reason = "tracking error 0.312 rad", .joint = 1 }, start);
    EXPECT_EQ("Joint 2 tracking error 0.312 rad", tracking.message);
    EXPECT_EQ(simulator::LogLevel::fault, tracking.level);
    EXPECT_EQ(simulator::LogLevel::warning, simulator::Describe({ .kind = ArmEventKind::moveAborted }, start).level);

    const auto misconfigured = simulator::Describe(
        { .kind = ArmEventKind::stateChanged, .reason = "reads back different", .joint = 4, .state = arm::ArmState::misconfigured }, start);
    EXPECT_EQ("Arm misconfigured: Joint 5 reads back different", misconfigured.message);
    EXPECT_EQ(simulator::LogLevel::fault, misconfigured.level);
}

TEST_F(EventLogTest, keeps_the_newest_lines_up_to_its_capacity)
{
    for (int line = 0; line != 4; ++line)
        log.Add(At({ .kind = ArmEventKind::moveDone }, std::chrono::seconds{ line }));

    ASSERT_EQ(3u, log.Lines().size());
    EXPECT_NEAR(1.0f, log.Lines().front().seconds, 1.0e-5f);
}

TEST_F(EventLogTest, formats_a_line_with_time_level_joint_and_message)
{
    log.Add(At({ .kind = ArmEventKind::jointSilent, .joint = 1 }, 1500ms));

    EXPECT_EQ("     1.500  Fault  J2  Joint 2 silent", log.Format(log.Lines().front()));
}

TEST_F(EventLogTest, filters_by_level_and_by_joint)
{
    log.Add(At({ .kind = ArmEventKind::moveDone }, 0ms));
    log.Add(At({ .kind = ArmEventKind::jointSilent, .joint = 1 }, 1ms));
    log.Add(At({ .kind = ArmEventKind::jointSilent, .joint = 4 }, 2ms));

    EXPECT_EQ(2, std::ranges::count(log.Text({ .minimum = simulator::LogLevel::warning }), '\n'));
    EXPECT_EQ(1, std::ranges::count(log.Text({ .joint = 4 }), '\n'));
    EXPECT_EQ(3, std::ranges::count(log.Text({}), '\n'));
}

TEST_F(EventLogTest, command_results_start_with_a_capital)
{
    EXPECT_EQ("Check contracts: accepted", simulator::Describe({ .kind = ArmEventKind::accepted, .commandName = "check contracts" }, start).message);
}

TEST_F(EventLogTest, a_note_from_the_simulator_is_logged_with_the_time_since_start)
{
    simulator::EventLog recent{ std::chrono::system_clock::now() - 2s };
    recent.Note(simulator::LogLevel::warning, 2, "Link 3 mesh not loaded: cannot open forearm.glb");

    const auto& line = recent.Lines().back();
    EXPECT_EQ(simulator::LogLevel::warning, line.level);
    EXPECT_EQ(2u, line.joint);
    EXPECT_EQ("Link 3 mesh not loaded: cannot open forearm.glb", line.message);
    EXPECT_NEAR(2.0f, line.seconds, 0.5f);
}

TEST_F(EventLogTest, lost_events_are_a_warning_that_says_how_many)
{
    const auto line = simulator::Describe(At({ .kind = ArmEventKind::eventsDropped, .dropped = 12 }, 2s), start);

    EXPECT_EQ(simulator::LogLevel::warning, line.level);
    EXPECT_EQ("12 events lost before this line: the log fell behind the arm", line.message);
    EXPECT_NEAR(2.0f, line.seconds, 1e-3f);
}

TEST_F(EventLogTest, a_trace_gap_is_logged_at_the_sample_after_it_with_its_length)
{
    const std::array samples{ arm_host::TraceSample{ .time = start + 3s, .droppedBefore = 0 }, arm_host::TraceSample{ .time = start + 5s, .droppedBefore = 40 },
        arm_host::TraceSample{ .time = start + 5s + 10ms } };

    EXPECT_EQ(1u, log.NoteTraceGaps(samples, 10ms));

    ASSERT_EQ(1u, log.Lines().size());
    const auto& line = log.Lines().front();
    EXPECT_EQ(simulator::LogLevel::warning, line.level);
    EXPECT_NEAR(5.0f, line.seconds, 1e-3f);
    EXPECT_EQ("Trace gap: 40 samples (about 400 ms) lost before this time; the scope and tuning skip them", line.message);
}

TEST_F(EventLogTest, a_trace_without_gaps_logs_nothing)
{
    const std::array samples{ arm_host::TraceSample{ .time = start + 1s }, arm_host::TraceSample{ .time = start + 1s + 10ms } };

    EXPECT_EQ(0u, log.NoteTraceGaps(samples, 10ms));

    EXPECT_TRUE(log.Lines().empty());
}

TEST_F(EventLogTest, names_each_arm_controller_change_and_warns_about_none_or_a_fallback)
{
    const auto changed = simulator::Describe({ .kind = ArmEventKind::armControllerChanged, .armController = "gravity-pid" }, start);
    EXPECT_EQ("Arm controller: gravity-pid", changed.message);
    EXPECT_EQ(simulator::LogLevel::info, changed.level);

    EXPECT_EQ(simulator::LogLevel::warning, simulator::Describe({ .kind = ArmEventKind::armControllerChanged, .armController = "none" }, start).level);

    const auto fallback = simulator::Describe(
        { .kind = ArmEventKind::armControllerChanged, .reason = "free is offered on a simulation only", .armController = "gravity-pd" }, start);
    EXPECT_EQ("Arm controller: gravity-pd (free is offered on a simulation only)", fallback.message);
    EXPECT_EQ(simulator::LogLevel::warning, fallback.level);
}
