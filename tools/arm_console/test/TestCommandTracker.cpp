#include "tools/arm_console/CommandTracker.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <thread>

namespace
{
    using namespace std::chrono_literals;
    using arm_host::ArmEventKind;

    class CommandTrackerTest
        : public testing::Test
    {
    public:
        arm_console::CommandTracker tracker;
    };
}

TEST_F(CommandTrackerTest, keeps_acceptance_refusal_and_no_answer_apart)
{
    tracker.Note({ .kind = ArmEventKind::accepted, .command = 1 });
    tracker.Note({ .kind = ArmEventKind::refused, .command = 2 });
    tracker.Note({ .kind = ArmEventKind::profileLoaded, .command = 3 });
    tracker.Note({ .kind = ArmEventKind::targetFailed, .command = 4 });

    EXPECT_EQ(arm_console::Answer::accepted, tracker.Await(1, 0ms));
    EXPECT_EQ(arm_console::Answer::refused, tracker.Await(2, 0ms));
    EXPECT_EQ(arm_console::Answer::accepted, tracker.Await(3, 0ms));
    EXPECT_EQ(arm_console::Answer::refused, tracker.Await(4, 0ms));
    EXPECT_EQ(arm_console::Answer::noAnswer, tracker.Await(5, 10ms));
}

TEST_F(CommandTrackerTest, an_answer_is_consumed_once_read)
{
    tracker.Note({ .kind = ArmEventKind::accepted, .command = 7 });
    EXPECT_EQ(1u, tracker.Held());

    EXPECT_EQ(arm_console::Answer::accepted, tracker.Await(7, 0ms));
    EXPECT_EQ(0u, tracker.Held());
    EXPECT_EQ(arm_console::Answer::noAnswer, tracker.Await(7, 0ms));
}

TEST_F(CommandTrackerTest, a_late_answer_is_dropped_instead_of_held)
{
    EXPECT_EQ(arm_console::Answer::noAnswer, tracker.Await(9, 0ms));

    tracker.Note({ .kind = ArmEventKind::accepted, .command = 9 });

    EXPECT_EQ(0u, tracker.Held());
}

TEST_F(CommandTrackerTest, waits_for_an_answer_posted_from_another_thread)
{
    std::jthread arm{ [this]()
        {
            std::this_thread::sleep_for(20ms);
            tracker.Note({ .kind = ArmEventKind::refused, .command = 4 });
        } };

    EXPECT_EQ(arm_console::Answer::refused, tracker.Await(4, 2000ms));
}

TEST_F(CommandTrackerTest, an_accepted_move_is_pending_until_it_ends)
{
    tracker.Note({ .kind = ArmEventKind::accepted, .command = 3, .commandName = "move" });
    EXPECT_EQ(arm_console::Answer::accepted, tracker.Await(3, 0ms));
    std::jthread arm{ [this]()
        {
            std::this_thread::sleep_for(30ms);
            tracker.Note({ .kind = ArmEventKind::moveAborted });
        } };

    EXPECT_EQ(arm_console::MoveEnd::aborted, tracker.AwaitMove(2000ms));
}

TEST_F(CommandTrackerTest, only_commands_that_move_make_a_move_pending)
{
    tracker.Note({ .kind = ArmEventKind::accepted, .command = 1, .commandName = "enable" });
    EXPECT_EQ(arm_console::MoveEnd::done, tracker.AwaitMove(0ms));

    for (const auto* name : { "move", "move through", "move line", "go to", "jog joint", "jog tool" })
    {
        tracker.Note({ .kind = ArmEventKind::accepted, .command = 2, .commandName = name });
        EXPECT_EQ(std::nullopt, tracker.AwaitMove(0ms)) << name;
        tracker.Note({ .kind = ArmEventKind::moveDone });
    }
}

TEST_F(CommandTrackerTest, waiting_with_no_move_pending_reports_the_last_move_or_done)
{
    EXPECT_EQ(arm_console::MoveEnd::done, tracker.AwaitMove(0ms));

    tracker.Note({ .kind = ArmEventKind::moveAborted });
    EXPECT_EQ(arm_console::MoveEnd::aborted, tracker.AwaitMove(0ms));

    tracker.Note({ .kind = ArmEventKind::moveDone });
    EXPECT_EQ(arm_console::MoveEnd::done, tracker.AwaitMove(0ms));
}

TEST_F(CommandTrackerTest, a_move_still_running_after_the_time_is_reported_as_no_end)
{
    tracker.Note({ .kind = ArmEventKind::accepted, .command = 1, .commandName = "move line" });

    EXPECT_EQ(std::nullopt, tracker.AwaitMove(30ms));
}

TEST_F(CommandTrackerTest, waits_for_a_lifecycle_state)
{
    EXPECT_TRUE(tracker.AwaitState(arm::ArmState::unpowered, 0ms).reached);
    std::jthread arm{ [this]()
        {
            std::this_thread::sleep_for(20ms);
            tracker.Note({ .kind = ArmEventKind::stateChanged, .state = arm::ArmState::checking });
            tracker.Note({ .kind = ArmEventKind::stateChanged, .state = arm::ArmState::ready });
        } };

    EXPECT_TRUE(tracker.AwaitState(arm::ArmState::ready, 2000ms).reached);
}

TEST_F(CommandTrackerTest, waiting_for_a_state_ends_early_on_a_trip_or_a_failed_configuration)
{
    tracker.Note({ .kind = ArmEventKind::stateChanged, .state = arm::ArmState::misconfigured });

    const auto waited = tracker.AwaitState(arm::ArmState::ready, 2000ms);

    EXPECT_FALSE(waited.reached);
    EXPECT_EQ(arm::ArmState::misconfigured, waited.state);
    EXPECT_FALSE(tracker.AwaitState(arm::ArmState::enabled, 10ms).reached);
}
