#include "infra/timer/test_helper/ClockFixture.hpp"
#include "tools/arm_host/ArmRuntime.hpp"
#include "tools/arm_host/test/ArmHostTestSupport.hpp"
#include "gmock/gmock.h"

namespace
{
    using namespace std::chrono_literals;
    using arm_host::ArmEventKind;
    using arm_host::ArmTarget;
    using arm_host::TargetKind;

    class ArmRuntimeTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        arm_host::TargetOpener Opener()
        {
            return [this](const ArmTarget& target) -> std::unique_ptr<arm_host::ArmTargetLink>
            {
                opened.push_back(target);
                if (target.kind == TargetKind::real)
                    return nullptr;
                return std::make_unique<arm_host::SimulatedTargetLink>(arm_host::test::nodes, 20ms);
            };
        }

        void Execute(uint32_t id, const arm_host::ArmCommand& command)
        {
            runtime->Execute(id, command);
            ForwardTime(30ms);
        }

        void StartMoving()
        {
            Execute(1, arm_host::CheckContracts{});
            Execute(2, arm_host::SelectMode{ can::FocMotorMode::position });
            Execute(3, arm_host::StartJoints{});
            arm::JointPositions target{};
            target[0] = foc::Radians{ 1.0f };
            Execute(4, arm_host::MoveTo{ target });
        }

        const ArmTarget simulated{ TargetKind::simulated, "" };
        const ArmTarget virtualBus{ TargetKind::virtualBus, "virtual:arm" };
        const ArmTarget realArm{ TargetKind::real, "socketcan:can0" };
        std::vector<ArmTarget> opened;
        arm_host::test::RecordingSink sink;
        std::optional<arm_host::ArmRuntime> runtime;
    };
}

TEST_F(ArmRuntimeTest, opens_its_initial_target_and_reports_it_ready)
{
    runtime.emplace(arm_host::test::Config(), Opener(), sink, simulated);

    ASSERT_NE(nullptr, sink.Find(ArmEventKind::targetReady));
    EXPECT_TRUE(sink.snapshot.targetReady);
    EXPECT_EQ(simulated, sink.snapshot.target);
}

TEST_F(ArmRuntimeTest, without_a_target_refuses_every_command)
{
    runtime.emplace(arm_host::test::Config(), Opener(), sink, realArm);
    Execute(5, arm_host::StartJoints{});

    ASSERT_NE(nullptr, sink.Find(ArmEventKind::targetFailed));
    EXPECT_FALSE(sink.snapshot.targetReady);
    EXPECT_EQ(ArmEventKind::refused, sink.Last()->kind);
    EXPECT_EQ("no target", sink.Last()->reason);
}

TEST_F(ArmRuntimeTest, refuses_to_switch_while_a_move_is_running)
{
    runtime.emplace(arm_host::test::Config(), Opener(), sink, simulated);
    StartMoving();
    runtime->SwitchTarget(9, virtualBus);

    EXPECT_EQ(ArmEventKind::targetFailed, sink.Last()->kind);
    EXPECT_EQ(9u, sink.Last()->command);
    EXPECT_EQ("moving", sink.Last()->reason);
    EXPECT_EQ(1u, opened.size());
}

TEST_F(ArmRuntimeTest, a_switch_stops_the_joints_waits_and_builds_the_new_target_stopped)
{
    runtime.emplace(arm_host::test::Config(), Opener(), sink, simulated);
    Execute(1, arm_host::CheckContracts{});
    runtime->SwitchTarget(9, ArmTarget{ TargetKind::simulated, "second" });

    ForwardTime(arm_host::ArmRuntime::stopGrace - 1ms);
    EXPECT_EQ(1u, opened.size());

    ForwardTime(1ms);
    ASSERT_EQ(2u, opened.size());
    EXPECT_EQ(ArmEventKind::targetReady, sink.Last()->kind);
    EXPECT_EQ(9u, sink.Last()->command);
    EXPECT_EQ("second", sink.snapshot.target.bus);
    EXPECT_EQ(can::ContractCompatibility::unknown, sink.snapshot.contracts);
}

TEST_F(ArmRuntimeTest, refuses_commands_while_switching)
{
    runtime.emplace(arm_host::test::Config(), Opener(), sink, simulated);
    runtime->SwitchTarget(9, simulated);
    runtime->Execute(10, arm_host::StartJoints{});

    EXPECT_EQ(ArmEventKind::refused, sink.Last()->kind);
    EXPECT_EQ("switching target", sink.Last()->reason);
}

TEST_F(ArmRuntimeTest, a_failed_switch_returns_to_the_previous_target)
{
    runtime.emplace(arm_host::test::Config(), Opener(), sink, simulated);
    runtime->SwitchTarget(9, realArm);
    ForwardTime(arm_host::ArmRuntime::stopGrace);

    const auto failed = std::ranges::find(sink.events, ArmEventKind::targetFailed, &arm_host::ArmEvent::kind);
    ASSERT_NE(sink.events.end(), failed);
    EXPECT_EQ(9u, failed->command);
    EXPECT_EQ(ArmEventKind::targetReady, sink.Last()->kind);
    EXPECT_EQ(simulated, sink.snapshot.target);
    EXPECT_TRUE(sink.snapshot.targetReady);
}
