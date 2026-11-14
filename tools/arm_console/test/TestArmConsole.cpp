#include "tools/arm_console/ArmConsole.hpp"
#include "tools/arm_host/DefaultArmConfig.hpp"
#include "tools/arm_host/ProfileFile.hpp"
#include "tools/arm_host/test_doubles/ArmCommandsMock.hpp"
#include "gmock/gmock.h"
#include <filesystem>
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
    snapshot.overruns = 4;
    snapshot.state = arm::ArmState::tripped;
    EXPECT_CALL(arm, Snapshot()).WillOnce(Return(snapshot));

    const auto status = Run("status");

    EXPECT_THAT(status, HasSubstr("joint 1: position 0.000 rad, speed 0.000 rad/s, state idle, fault none\n"));
    EXPECT_THAT(status, HasSubstr("joint 4: position 0.250 rad, speed -1.500 rad/s, state fault, fault overCurrent\n"));
    EXPECT_THAT(status, HasSubstr("joint 6: position 0.000 rad, speed 0.000 rad/s, state idle, fault none, silent\n"));
    EXPECT_THAT(status, HasSubstr("safety: tripped\n"));
    EXPECT_THAT(status, HasSubstr("motion: idle\n"));
    EXPECT_THAT(status, HasSubstr("overruns: 4\n"));
    EXPECT_THAT(status, HasSubstr("arm: tripped\n"));
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

TEST_F(ArmConsoleTest, controllers_lists_the_arm_controllers_and_controller_posts_by_name)
{
    arm_host::ArmSnapshot snapshot;
    snapshot.armControllers = { { "pass-through", true, true }, { "smoothed", false, false } };
    EXPECT_CALL(arm, Snapshot()).WillRepeatedly(Return(snapshot));

    EXPECT_EQ("* pass-through\n  smoothed (disabled)\n", Run("controllers"));

    EXPECT_CALL(arm, Post(VariantWith<arm_host::EnableArmController>(Field(&arm_host::EnableArmController::id, 1)))).WillOnce(Return(4));
    EXPECT_CALL(arm, Post(VariantWith<arm_host::SelectArmController>(Field(&arm_host::SelectArmController::id, 1)))).WillOnce(Return(5));
    EXPECT_CALL(arm, Post(VariantWith<arm_host::DisableArmController>(Field(&arm_host::DisableArmController::id, 0)))).WillOnce(Return(6));
    EXPECT_EQ(4u, console.Execute("controller enable smoothed").command);
    EXPECT_EQ(5u, console.Execute("controller select smoothed").command);
    EXPECT_EQ(6u, console.Execute("controller disable pass-through").command);

    EXPECT_EQ("no arm controller lqr\n", Run("controller select lqr"));
    EXPECT_THAT(Run("controller toggle smoothed"), StartsWith("usage:"));
    EXPECT_EQ("arm controller: smoothed\n", Printed({ .kind = ArmEventKind::armControllerChanged, .armController = "smoothed" }));
}

TEST_F(ArmConsoleTest, quit_ends_the_console)
{
    EXPECT_EQ(arm_console::Outcome::quit, console.Execute("quit").outcome);
}

TEST_F(ArmConsoleTest, load_reads_a_profile_file_and_save_writes_the_profile_in_effect)
{
    const auto path = std::filesystem::temp_directory_path() / "e-robot-arm-console-test.json";
    arm_host::ArmSnapshot snapshot{};
    snapshot.profileName = "bench arm";
    snapshot.config = arm_host::DefaultArmConfig({ 31, 32, 33, 34, 35, 36 });
    EXPECT_CALL(arm, Snapshot()).WillRepeatedly(Return(snapshot));

    EXPECT_EQ("saved " + path.string() + "\n", Run("save " + path.string()));

    EXPECT_CALL(arm, LoadProfile(_)).WillOnce(Invoke([](const arm::ArmProfile& profile)
        {
            EXPECT_EQ("bench arm", std::string(profile.name.begin(), profile.name.end()));
            EXPECT_EQ(31, profile.config.nodes[0]);
            return 12u;
        }));
    EXPECT_EQ(12u, console.Execute("load " + path.string()).command);
    EXPECT_EQ("profile loaded: bench arm\n", Printed({ .kind = ArmEventKind::profileLoaded, .command = 12 }));
    std::filesystem::remove(path);
}

TEST_F(ArmConsoleTest, a_profile_that_cannot_be_read_is_refused_with_the_reason)
{
    EXPECT_EQ("profile refused: cannot open /no/such/profile.json\n", Run("load /no/such/profile.json"));
    EXPECT_THAT(Run("load"), HasSubstr("usage:"));
}

TEST_F(ArmConsoleTest, pose_gravity_fk_and_ik_convert_without_moving_the_arm)
{
    arm_host::ArmSnapshot snapshot{};
    snapshot.config = arm_host::DefaultArmConfig({ 1, 2, 3, 4, 5, 6 });
    snapshot.toolPose = { 0.02f, 0.0f, 0.71f, 0.0f, 0.0f, 0.0f };
    snapshot.singularities = { true, true, false };
    snapshot.gravityTorque = { foc::NewtonMeter{ 0.0f }, foc::NewtonMeter{ -7.0157f }, foc::NewtonMeter{ -0.0001f }, foc::NewtonMeter{ 0.0f },
        foc::NewtonMeter{ 0.25f }, foc::NewtonMeter{ 0.0f } };
    EXPECT_CALL(arm, Snapshot()).WillRepeatedly(Return(snapshot));

    EXPECT_EQ("gravity torque: 0.000 -7.016 0.000 0.000 0.250 0.000 N·m\n", Run("gravity"));
    EXPECT_THAT(Run("gravity 1"), HasSubstr("usage:"));
    EXPECT_EQ("tool: x 0.020 y 0.000 z 0.710 m, roll 0.000 pitch 0.000 yaw 0.000 rad, near wrist elbow singularity\n", Run("pose"));
    EXPECT_EQ("tool: x 0.580 y 0.000 z 0.150 m, roll 0.000 pitch 1.571 yaw 0.000 rad, near wrist elbow singularity\n", Run("fk 0 1.5707963 0 0 0 0"));
    EXPECT_EQ("tool: x 0.562 y 0.000 z 0.258 m, roll 0.000 pitch 1.571 yaw 0.000 rad\n", Run("fk 0 1.2 0.3 0 0.0707963 0"));
    EXPECT_THAT(Run("ik 0.4 0.1 0.3 0 1.5707963 0"), StartsWith("joints:"));
    EXPECT_EQ("ik refused: unreachable\n", Run("ik 2 0 0 0 0 0"));
    EXPECT_THAT(Run("fk 1 2"), HasSubstr("usage:"));
}
