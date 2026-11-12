#include "infra/timer/test_helper/ClockFixture.hpp"
#include "tools/arm_host/ArmSession.hpp"
#include "tools/arm_host/test/ArmHostTestSupport.hpp"
#include "gmock/gmock.h"

namespace
{
    using namespace std::chrono_literals;
    using arm_host::ArmEventKind;

    class ArmSessionTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        void Execute(uint32_t id, const arm_host::ArmCommand& command)
        {
            session.Execute(id, command);
            ForwardTime(30ms);
        }

        void BringUp()
        {
            Execute(1, arm_host::CheckContracts{});
            Execute(2, arm_host::SelectMode{ can::FocMotorMode::position });
            Execute(3, arm_host::StartJoints{});
        }

        arm::JointPositions Target(float first) const
        {
            arm::JointPositions target{};
            target[0] = foc::Radians{ first };
            return target;
        }

        arm_host::ArmTarget target{ arm_host::TargetKind::simulated, "" };
        simulation::SimulatedArm simulatedArm{ arm_host::test::nodes, 50ms };
        arm_host::test::RecordingSink sink;
        arm_host::ArmSession session{ simulatedArm.ArmSide(), arm_host::test::Config(), target, sink, &simulatedArm };
    };
}

TEST_F(ArmSessionTest, publishes_a_ready_snapshot_of_its_target_at_once)
{
    EXPECT_EQ(1u, sink.snapshots);
    EXPECT_TRUE(sink.snapshot.targetReady);
    EXPECT_EQ(target, sink.snapshot.target);
    EXPECT_FALSE(sink.snapshot.commanded.has_value());
}

TEST_F(ArmSessionTest, publishes_a_snapshot_and_a_trace_sample_every_control_period)
{
    ForwardTime(100ms);

    EXPECT_EQ(11u, sink.snapshots);
    EXPECT_EQ(11u, sink.trace.size());
}

TEST_F(ArmSessionTest, reports_an_accepted_command_with_its_id_and_name)
{
    Execute(7, arm_host::CheckContracts{});

    ASSERT_NE(nullptr, sink.Find(ArmEventKind::accepted));
    EXPECT_EQ(7u, sink.Find(ArmEventKind::accepted)->command);
    EXPECT_EQ("check contracts", sink.Find(ArmEventKind::accepted)->commandName);
    ASSERT_NE(nullptr, sink.Find(ArmEventKind::contractsResolved));
    EXPECT_EQ(can::ContractCompatibility::compatible, sink.Find(ArmEventKind::contractsResolved)->contracts);
}

TEST_F(ArmSessionTest, refuses_motion_before_the_contracts_are_checked_and_says_why)
{
    Execute(4, arm_host::StartJoints{});

    ASSERT_NE(nullptr, sink.Find(ArmEventKind::refused));
    EXPECT_EQ(4u, sink.Find(ArmEventKind::refused)->command);
    EXPECT_EQ("contracts not compatible", sink.Find(ArmEventKind::refused)->reason);
}

TEST_F(ArmSessionTest, a_move_shows_the_commanded_positions_and_the_joints_follow_with_a_lag)
{
    BringUp();
    Execute(5, arm_host::MoveTo{ Target(0.5f) });
    ForwardTime(2s);

    ASSERT_NE(nullptr, sink.Find(ArmEventKind::moveDone));
    ASSERT_TRUE(sink.snapshot.commanded.has_value());
    EXPECT_NEAR(0.5f, sink.snapshot.commanded->at(0).Value(), 1.0e-4f);
    EXPECT_NEAR(0.5f, sink.snapshot.readings[0].position.Value(), 1.0e-3f);
    EXPECT_NEAR(0.5f, sink.trace.back().commanded[0].Value(), 1.0e-4f);
}

TEST_F(ArmSessionTest, a_lagging_joint_trails_its_setpoint_while_moving)
{
    BringUp();
    Execute(5, arm_host::MoveTo{ Target(1.0f) });
    ForwardTime(500ms);

    ASSERT_TRUE(sink.snapshot.commanded.has_value());
    EXPECT_GT(sink.snapshot.commanded->at(0).Value(), sink.snapshot.readings[0].position.Value());
    EXPECT_GT(sink.snapshot.readings[0].speed.Value(), 0.0f);
}

TEST_F(ArmSessionTest, an_injected_fault_trips_the_arm_and_names_the_joint)
{
    BringUp();
    Execute(6, arm_host::InjectFault{ 2, can::FocFaultCode::overCurrent });

    ASSERT_NE(nullptr, sink.Find(ArmEventKind::tripped));
    EXPECT_EQ(2u, sink.Find(ArmEventKind::tripped)->joint);
    EXPECT_EQ(can::FocFaultCode::overCurrent, sink.Find(ArmEventKind::tripped)->fault);
    EXPECT_TRUE(sink.snapshot.tripped);
}

TEST_F(ArmSessionTest, a_silenced_joint_trips_the_arm_once)
{
    BringUp();
    Execute(6, arm_host::SilenceJoint{ 4 });
    ForwardTime(100ms);

    EXPECT_EQ(1, std::ranges::count(sink.events, ArmEventKind::tripped, &arm_host::ArmEvent::kind));
    EXPECT_EQ(4u, sink.Find(ArmEventKind::tripped)->joint);
}

TEST_F(ArmSessionTest, refuses_a_failure_on_a_joint_that_does_not_exist)
{
    Execute(6, arm_host::InjectFault{ 6, can::FocFaultCode::overCurrent });

    ASSERT_NE(nullptr, sink.Last());
    EXPECT_EQ(ArmEventKind::refused, sink.Last()->kind);
    EXPECT_EQ("no such joint", sink.Last()->reason);
}

TEST_F(ArmSessionTest, refuses_to_rearm_while_a_joint_is_in_fault_and_rearms_once_cleared)
{
    BringUp();
    Execute(6, arm_host::InjectFault{ 0, can::FocFaultCode::stall });
    Execute(7, arm_host::Rearm{});

    ASSERT_NE(nullptr, sink.Last());
    EXPECT_EQ("a joint is in fault", sink.Last()->reason);

    Execute(8, arm_host::ClearFaults{});
    Execute(9, arm_host::Rearm{});
    ForwardTime(20ms);

    EXPECT_EQ(9u, sink.events.back().command);
    EXPECT_EQ(ArmEventKind::accepted, sink.events.back().kind);
    EXPECT_FALSE(sink.snapshot.tripped);
}

TEST_F(ArmSessionTest, emergency_stop_aborts_a_running_move)
{
    BringUp();
    Execute(5, arm_host::MoveTo{ Target(1.0f) });
    Execute(6, arm_host::EmergencyStop{});

    EXPECT_NE(nullptr, sink.Find(ArmEventKind::moveAborted));
    EXPECT_FALSE(sink.snapshot.moving);
    EXPECT_EQ(1u, simulatedArm.Joint(0).EmergencyStops());
}

TEST_F(ArmSessionTest, commands_replaced_before_every_joint_accepted_them_are_refused_and_the_newest_is_accepted)
{
    session.Execute(1, arm_host::StopJoints{});
    session.Execute(2, arm_host::StopJoints{});
    session.Execute(3, arm_host::ClearFaults{});
    ForwardTime(30ms);

    std::vector<uint32_t> replaced;
    for (const auto& event : sink.events)
        if (event.kind == ArmEventKind::refused && event.reason == "replaced by a newer command")
            replaced.push_back(event.command);

    EXPECT_EQ((std::vector<uint32_t>{ 1, 2 }), replaced);
    EXPECT_EQ(3u, sink.events.back().command);
    EXPECT_EQ(ArmEventKind::accepted, sink.events.back().kind);
}
