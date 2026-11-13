#include "infra/timer/test_helper/ClockFixture.hpp"
#include "tools/arm_host/ArmSession.hpp"
#include "tools/arm_host/test/ArmHostTestSupport.hpp"
#include "gmock/gmock.h"
#include <algorithm>
#include <vector>

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
        arm_host::ArmSession session{ simulatedArm.ArmSide(), arm_host::test::Profile(), target, sink, &simulatedArm };
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

    EXPECT_TRUE(std::ranges::any_of(sink.events, [](const arm_host::ArmEvent& event)
        {
            return event.command == 9 && event.kind == ArmEventKind::accepted;
        }));
    EXPECT_FALSE(sink.snapshot.tripped);
    EXPECT_EQ(arm::ArmState::ready, sink.snapshot.state);
}

TEST_F(ArmSessionTest, bring_up_and_enable_walk_the_lifecycle_and_report_each_state)
{
    Execute(1, arm_host::BringUp{});
    EXPECT_EQ(arm::ArmState::ready, sink.snapshot.state);

    Execute(2, arm_host::Enable{});
    EXPECT_EQ(arm::ArmState::enabled, sink.snapshot.state);
    EXPECT_EQ(can::FocMotorMode::position, simulatedArm.Joint(5).Mode());

    std::vector<arm::ArmState> states;
    for (const auto& event : sink.events)
        if (event.kind == ArmEventKind::stateChanged)
            states.push_back(event.state);
    EXPECT_EQ((std::vector<arm::ArmState>{ arm::ArmState::checking, arm::ArmState::waitingForCalibration, arm::ArmState::ready, arm::ArmState::enabling,
                  arm::ArmState::enabled }),
        states);
}

TEST_F(ArmSessionTest, refuses_lifecycle_commands_and_moves_out_of_order_and_names_the_state)
{
    Execute(1, arm_host::Enable{});
    EXPECT_EQ("unpowered", sink.Last()->reason);

    Execute(2, arm_host::CheckContracts{});
    Execute(3, arm_host::MoveTo{ Target(0.5f) });
    EXPECT_EQ(3u, sink.Last()->command);
    EXPECT_EQ("the arm is not enabled", sink.Last()->reason);
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

TEST_F(ArmSessionTest, applies_and_reads_a_joint_controller_once_the_arm_is_ready)
{
    arm::ControllerSettings settings{};
    settings.speed = can::FocSpeedAlgorithm::adrc;
    settings[arm::Gain::speedObserverBandwidthRatio] = 8.0f;

    Execute(1, arm_host::ApplyController{ 3, settings });
    EXPECT_EQ("the arm is not ready", sink.Last()->reason);

    Execute(2, arm_host::BringUp{});
    Execute(3, arm_host::ApplyController{ 3, settings });
    ASSERT_NE(nullptr, sink.Find(ArmEventKind::controllerApplied));
    EXPECT_EQ(3u, sink.Find(ArmEventKind::controllerApplied)->joint);
    EXPECT_FALSE(arm::Compare(settings, simulatedArm.Joint(3).Controller()).Any());

    Execute(4, arm_host::ReadController{ 3 });
    ASSERT_NE(nullptr, sink.Find(ArmEventKind::controllerRead));
    EXPECT_FALSE(arm::Compare(settings, sink.Find(ArmEventKind::controllerRead)->controller.settings).Any());
}

namespace
{
    arm::ArmConfig ConfigWithController()
    {
        auto config = arm_host::test::Config();
        arm::ControllerSettings settings{};
        settings[arm::Gain::speedBandwidth] = 120.0f;
        config.controllers[1] = settings;
        return config;
    }

    class ArmSessionConfiguringTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        arm_host::ArmTarget target{ arm_host::TargetKind::simulated, "" };
        simulation::SimulatedArm simulatedArm{ arm_host::test::nodes, 50ms };
        arm_host::test::RecordingSink sink;
        arm_host::ArmSession session{ simulatedArm.ArmSide(), arm_host::test::Profile(ConfigWithController()), target, sink, &simulatedArm };
    };
}

TEST_F(ArmSessionConfiguringTest, names_the_joint_whose_motor_controller_reads_back_different)
{
    simulatedArm.Joint(1).IgnoreParameter(can::FocParameter::speedBandwidth);
    session.Execute(1, arm_host::BringUp{});
    ForwardTime(500ms);

    EXPECT_EQ(arm::ArmState::misconfigured, sink.snapshot.state);
    EXPECT_EQ((arm::Misconfiguration{ 1, arm::MisconfigurationReason::differs }), sink.snapshot.misconfiguration);
    const auto event = std::ranges::find_if(sink.events, [](const arm_host::ArmEvent& candidate)
        {
            return candidate.kind == ArmEventKind::stateChanged && candidate.state == arm::ArmState::misconfigured;
        });
    ASSERT_NE(sink.events.end(), event);
    EXPECT_EQ(1u, event->joint);
    EXPECT_EQ("reads back different", event->reason);
}
