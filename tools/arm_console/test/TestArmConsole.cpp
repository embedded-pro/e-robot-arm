#include "core/motion/test_doubles/JointMotionMock.hpp"
#include "core/safety/test_doubles/SafetySupervisorMock.hpp"
#include "tools/arm_console/ArmConsole.hpp"
#include "gmock/gmock.h"
#include <sstream>

namespace
{
    using namespace testing;

    class ArmConsoleTest
        : public Test
    {
    public:
        std::string Run(const std::string& line)
        {
            output.str("");
            EXPECT_TRUE(console.Execute(line));
            return output.str();
        }

        StrictMock<arm::SafetySupervisorMock> safety;
        StrictMock<arm::JointMotionMock> motion;
        std::ostringstream output;
        arm_console::ArmConsole console{ safety, motion, output };
    };
}

TEST_F(ArmConsoleTest, joint_bus_commands_reach_the_safety_supervisor_and_report_acceptance)
{
    InSequence order;
    EXPECT_CALL(safety, QueryContracts()).WillOnce(Return(true));
    EXPECT_CALL(safety, Start()).WillOnce(Return(false));
    EXPECT_CALL(safety, Stop()).WillOnce(Return(true));
    EXPECT_CALL(safety, EmergencyStop()).WillOnce(Return(true));
    EXPECT_CALL(safety, ClearFault()).WillOnce(Return(true));
    EXPECT_CALL(safety, Rearm()).WillOnce(Return(false));

    EXPECT_EQ("ok\n", Run("contracts"));
    EXPECT_EQ("refused\n", Run("start"));
    EXPECT_EQ("ok\n", Run("stop"));
    EXPECT_EQ("ok\n", Run("estop"));
    EXPECT_EQ("ok\n", Run("clear"));
    EXPECT_EQ("refused\n", Run("rearm"));
}

TEST_F(ArmConsoleTest, mode_selects_the_named_control_mode)
{
    EXPECT_CALL(safety, SelectMode(can::FocMotorMode::position)).WillOnce(Return(true));
    EXPECT_CALL(safety, SelectMode(can::FocMotorMode::torque)).WillOnce(Return(true));

    EXPECT_EQ("ok\n", Run("mode position"));
    EXPECT_EQ("ok\n", Run("mode torque"));
}

TEST_F(ArmConsoleTest, move_sends_the_six_positions_to_joint_motion)
{
    arm::JointPositions target{ foc::Radians{ 0.1f }, foc::Radians{ -0.2f }, foc::Radians{ 0.3f },
        foc::Radians{ 0.0f }, foc::Radians{ 1.5f }, foc::Radians{ -1.0f } };
    EXPECT_CALL(motion, MoveTo(target)).WillOnce(Return(true));
    EXPECT_CALL(motion, Abort());

    EXPECT_EQ("ok\n", Run("move 0.1 -0.2 0.3 0 1.5 -1"));
    EXPECT_EQ("ok\n", Run("abort"));
}

TEST_F(ArmConsoleTest, a_malformed_or_unknown_command_prints_the_usage_and_sends_nothing)
{
    EXPECT_THAT(Run("move 1 2 3"), StartsWith("usage:"));
    EXPECT_THAT(Run("move 1 2 3 4 5 six"), StartsWith("usage:"));
    EXPECT_THAT(Run("mode fast"), StartsWith("usage:"));
    EXPECT_THAT(Run("jump"), StartsWith("usage:"));
    EXPECT_EQ("", Run(""));
}

TEST_F(ArmConsoleTest, status_prints_every_joint_the_trip_and_the_move)
{
    std::array<arm::JointReading, arm::jointCount> readings{};
    readings[3] = { foc::Radians{ 0.25f }, foc::RadiansPerSecond{ -1.5f }, can::FocMotorState::fault, can::FocFaultCode::overCurrent };
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_CALL(safety, Reading(joint)).WillRepeatedly(ReturnRef(readings[joint]));
    EXPECT_CALL(safety, Tripped()).WillOnce(Return(true));
    EXPECT_CALL(motion, Moving()).WillOnce(Return(false));

    const auto status = Run("status");

    EXPECT_THAT(status, HasSubstr("joint 1: position 0.000 rad, speed 0.000 rad/s, state idle, fault none\n"));
    EXPECT_THAT(status, HasSubstr("joint 4: position 0.250 rad, speed -1.500 rad/s, state fault, fault overCurrent\n"));
    EXPECT_THAT(status, HasSubstr("safety: tripped\n"));
    EXPECT_THAT(status, HasSubstr("motion: idle\n"));
}

TEST_F(ArmConsoleTest, events_are_printed_as_they_happen)
{
    safety.GetObserver().OnContractsResolved(can::ContractCompatibility::compatible);
    safety.GetObserver().OnContractsResolved(can::ContractCompatibility::incompatible);
    safety.GetObserver().OnJointFault(2, can::FocFaultCode::stall);
    safety.GetObserver().OnJointSilent(5);
    motion.GetObserver().OnMoveDone();
    motion.GetObserver().OnMoveAborted();

    EXPECT_EQ("contracts compatible\ncontracts incompatible\njoint 3 faulted: stall\njoint 6 silent\nmove done\nmove aborted\n", output.str());
}

TEST_F(ArmConsoleTest, quit_ends_the_console)
{
    EXPECT_FALSE(console.Execute("quit"));
}
