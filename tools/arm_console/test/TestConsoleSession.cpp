#include "tools/arm_console/ConsoleSession.hpp"
#include "tools/arm_host/test_doubles/ArmCommandsMock.hpp"
#include "gmock/gmock.h"
#include <sstream>

namespace
{
    using namespace testing;
    using arm_host::ArmEventKind;

    class ConsoleSessionTest
        : public Test
    {
    public:
        arm_console::ConsoleSession Session(bool batch)
        {
            return { console, tracker, output, batch };
        }

        StrictMock<arm_host::ArmCommandsMock> arm;
        std::ostringstream output;
        arm_console::ArmConsole console{ arm, output };
        arm_console::CommandTracker tracker;
    };
}

TEST_F(ConsoleSessionTest, a_batch_stops_at_the_first_refused_command_and_exits_with_failure)
{
    auto session = Session(true);
    EXPECT_CALL(arm, Post(_)).WillOnce(Return(1u)).WillOnce(Return(2u));
    tracker.Note({ .kind = ArmEventKind::accepted, .command = 1 });
    tracker.Note({ .kind = ArmEventKind::refused, .command = 2 });

    EXPECT_TRUE(session.Handle("bringup"));
    EXPECT_FALSE(session.Handle("enable"));

    EXPECT_EQ(1, session.ExitStatus());
    EXPECT_THAT(output.str(), HasSubstr("batch stopped at line 2: enable"));
}

TEST_F(ConsoleSessionTest, a_batch_stops_at_a_command_with_no_answer)
{
    auto session = Session(true);
    EXPECT_CALL(arm, Post(_)).WillOnce(Return(5u));

    EXPECT_FALSE(session.Handle("bringup"));

    EXPECT_THAT(output.str(), HasSubstr("no answer in 2 s: bringup"));
    EXPECT_EQ(1, session.ExitStatus());
}

TEST_F(ConsoleSessionTest, a_batch_stops_at_a_malformed_command_without_posting_it)
{
    auto session = Session(true);

    EXPECT_FALSE(session.Handle("move 1 2"));

    EXPECT_EQ(1, session.ExitStatus());
}

TEST_F(ConsoleSessionTest, a_batch_that_reaches_its_end_succeeds)
{
    auto session = Session(true);
    EXPECT_CALL(arm, Post(_)).WillOnce(Return(1u));
    tracker.Note({ .kind = ArmEventKind::accepted, .command = 1 });

    EXPECT_TRUE(session.Handle("bringup"));
    EXPECT_TRUE(session.Handle(""));

    EXPECT_EQ(0, session.ExitStatus());
}

TEST_F(ConsoleSessionTest, interactively_a_failure_is_reported_and_the_session_goes_on)
{
    auto session = Session(false);
    EXPECT_CALL(arm, Post(_)).WillOnce(Return(2u));
    tracker.Note({ .kind = ArmEventKind::refused, .command = 2 });

    EXPECT_TRUE(session.Handle("enable"));
    EXPECT_TRUE(session.Handle("nonsense"));

    EXPECT_EQ(0, session.ExitStatus());
    EXPECT_THAT(output.str(), Not(HasSubstr("batch stopped")));
}

TEST_F(ConsoleSessionTest, quit_ends_the_session)
{
    auto session = Session(true);

    EXPECT_FALSE(session.Handle("quit"));
    EXPECT_EQ(0, session.ExitStatus());
}

TEST_F(ConsoleSessionTest, wait_succeeds_when_the_move_ends_done_and_fails_when_it_was_aborted)
{
    auto session = Session(true);

    tracker.Note({ .kind = ArmEventKind::moveDone });
    EXPECT_TRUE(session.Handle("wait 1"));

    tracker.Note({ .kind = ArmEventKind::moveAborted });
    EXPECT_FALSE(session.Handle("wait"));
    EXPECT_THAT(output.str(), HasSubstr("batch stopped at line 2: wait"));
}

TEST_F(ConsoleSessionTest, wait_fails_when_the_move_is_still_running_after_its_time)
{
    auto session = Session(true);
    EXPECT_CALL(arm, Post(_)).WillOnce(Return(3u));
    tracker.Note({ .kind = ArmEventKind::accepted, .command = 3, .commandName = "move" });

    EXPECT_TRUE(session.Handle("move 0 0.5 0 0 0 0"));
    EXPECT_FALSE(session.Handle("wait 0.05"));

    EXPECT_THAT(output.str(), HasSubstr("wait: move still running after 0.05 s"));
}

TEST_F(ConsoleSessionTest, wait_for_a_state_succeeds_once_the_arm_is_in_it_and_names_where_it_stopped_otherwise)
{
    auto session = Session(true);
    tracker.Note({ .kind = ArmEventKind::stateChanged, .state = arm::ArmState::ready });

    EXPECT_TRUE(session.Handle("wait ready 1"));
    EXPECT_FALSE(session.Handle("wait enabled 0.05"));

    EXPECT_THAT(output.str(), HasSubstr("wait: arm ready, not enabled"));
}
