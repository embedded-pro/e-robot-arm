#include "tools/arm_console/ArmConsole.hpp"
#include "tools/arm_host/test_doubles/ArmCommandsMock.hpp"
#include "gmock/gmock.h"
#include <sstream>

namespace
{
    using namespace testing;
    using arm_host::ArmEventKind;

    MATCHER_P(Holds, type, "")
    {
        return std::holds_alternative<std::remove_cvref_t<decltype(type)>>(arg);
    }

    class ArmConsoleTest
        : public Test
    {
    public:
        std::string Run(const std::string& line, arm_console::Outcome expected = arm_console::Outcome::done)
        {
            output.str("");
            EXPECT_EQ(expected, console.Execute(line).outcome);
            return output.str();
        }

        std::string Printed(const arm_host::ArmEvent& event)
        {
            output.str("");
            console.Print(event);
            return output.str();
        }

        StrictMock<arm_host::ArmCommandsMock> arm;
        std::ostringstream output;
        arm_console::ArmConsole console{ arm, output };
    };
}

TEST_F(ArmConsoleTest, each_arm_command_is_posted_and_its_id_returned)
{
    InSequence order;
    EXPECT_CALL(arm, Post(Holds(arm_host::CheckContracts{}))).WillOnce(Return(1));
    EXPECT_CALL(arm, Post(Holds(arm_host::StartJoints{}))).WillOnce(Return(2));
    EXPECT_CALL(arm, Post(Holds(arm_host::StopJoints{}))).WillOnce(Return(3));
    EXPECT_CALL(arm, Post(Holds(arm_host::EmergencyStop{}))).WillOnce(Return(4));
    EXPECT_CALL(arm, Post(Holds(arm_host::ClearFaults{}))).WillOnce(Return(5));
    EXPECT_CALL(arm, Post(Holds(arm_host::Rearm{}))).WillOnce(Return(6));
    EXPECT_CALL(arm, Post(Holds(arm_host::AbortMove{}))).WillOnce(Return(7));

    uint32_t expectedId = 1;
    for (const auto* line : { "contracts", "start", "stop", "estop", "clear", "rearm", "abort" })
    {
        const auto result = console.Execute(line);
        EXPECT_EQ(arm_console::Outcome::posted, result.outcome);
        EXPECT_EQ(expectedId++, result.command);
    }
}

TEST_F(ArmConsoleTest, mode_posts_the_named_control_mode)
{
    EXPECT_CALL(arm, Post(Holds(arm_host::SelectMode{})))
        .WillOnce(Invoke([](const arm_host::ArmCommand& command)
            {
                EXPECT_EQ(can::FocMotorMode::position, std::get<arm_host::SelectMode>(command).mode);
                return 1u;
            }));

    Run("mode position", arm_console::Outcome::posted);
}

TEST_F(ArmConsoleTest, move_posts_the_six_positions)
{
    const arm::JointPositions target{ foc::Radians{ 0.1f }, foc::Radians{ -0.2f }, foc::Radians{ 0.3f },
        foc::Radians{ 0.0f }, foc::Radians{ 1.5f }, foc::Radians{ -1.0f } };
    EXPECT_CALL(arm, Post(Holds(arm_host::MoveTo{})))
        .WillOnce(Invoke([&target](const arm_host::ArmCommand& command)
            {
                EXPECT_EQ(target, std::get<arm_host::MoveTo>(command).target);
                return 1u;
            }));

    Run("move 0.1 -0.2 0.3 0 1.5 -1", arm_console::Outcome::posted);
}

TEST_F(ArmConsoleTest, a_malformed_or_unknown_command_prints_the_usage_and_posts_nothing)
{
    EXPECT_THAT(Run("move 1 2 3"), StartsWith("usage:"));
    EXPECT_THAT(Run("move 1 2 3 4 5 six"), StartsWith("usage:"));
    EXPECT_THAT(Run("mode fast"), StartsWith("usage:"));
    EXPECT_THAT(Run("jump"), StartsWith("usage:"));
    EXPECT_EQ("", Run(""));
}

TEST_F(ArmConsoleTest, status_prints_every_joint_the_trip_and_the_move_from_the_snapshot)
{
    arm_host::ArmSnapshot snapshot{};
    snapshot.readings[3] = { foc::Radians{ 0.25f }, foc::RadiansPerSecond{ -1.5f }, can::FocMotorState::fault, can::FocFaultCode::overCurrent };
    snapshot.readings[5].silent = true;
    snapshot.tripped = true;
    EXPECT_CALL(arm, Snapshot()).WillOnce(Return(snapshot));

    const auto status = Run("status");

    EXPECT_THAT(status, HasSubstr("joint 1: position 0.000 rad, speed 0.000 rad/s, state idle, fault none\n"));
    EXPECT_THAT(status, HasSubstr("joint 4: position 0.250 rad, speed -1.500 rad/s, state fault, fault overCurrent\n"));
    EXPECT_THAT(status, HasSubstr("joint 6: position 0.000 rad, speed 0.000 rad/s, state idle, fault none, silent\n"));
    EXPECT_THAT(status, HasSubstr("safety: tripped\n"));
    EXPECT_THAT(status, HasSubstr("motion: idle\n"));
}

TEST_F(ArmConsoleTest, events_print_as_one_line_each_with_joints_numbered_from_one)
{
    EXPECT_EQ("ok\n", Printed({ .kind = ArmEventKind::accepted }));
    EXPECT_EQ("refused: tripped\n", Printed({ .kind = ArmEventKind::refused, .reason = "tripped" }));
    EXPECT_EQ("contracts compatible\n", Printed({ .kind = ArmEventKind::contractsResolved, .contracts = can::ContractCompatibility::compatible }));
    EXPECT_EQ("contracts incompatible\n", Printed({ .kind = ArmEventKind::contractsResolved, .contracts = can::ContractCompatibility::incompatible }));
    EXPECT_EQ("joint 3 faulted: stall\n", Printed({ .kind = ArmEventKind::jointFault, .joint = 2, .fault = can::FocFaultCode::stall }));
    EXPECT_EQ("joint 6 silent\n", Printed({ .kind = ArmEventKind::jointSilent, .joint = 5 }));
    EXPECT_EQ("tripped by joint 6\n", Printed({ .kind = ArmEventKind::tripped, .joint = 5 }));
    EXPECT_EQ("move done\n", Printed({ .kind = ArmEventKind::moveDone }));
    EXPECT_EQ("move aborted\n", Printed({ .kind = ArmEventKind::moveAborted }));
    EXPECT_EQ("target ready\n", Printed({ .kind = ArmEventKind::targetReady }));
    EXPECT_EQ("target failed: moving\n", Printed({ .kind = ArmEventKind::targetFailed, .reason = "moving" }));
}

TEST_F(ArmConsoleTest, quit_ends_the_console)
{
    EXPECT_EQ(arm_console::Outcome::quit, console.Execute("quit").outcome);
}
