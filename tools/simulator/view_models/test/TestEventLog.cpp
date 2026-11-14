#include "tools/simulator/view_models/EventLog.hpp"
#include "gmock/gmock.h"
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
