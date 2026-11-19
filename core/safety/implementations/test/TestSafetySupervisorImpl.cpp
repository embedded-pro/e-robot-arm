#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "core/safety/implementations/SafetySupervisorImpl.hpp"
#include "core/safety/test_doubles/HardwareStopMock.hpp"
#include "core/safety/test_doubles/PoseGuardMock.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <limits>

namespace
{
    using namespace testing;
    using namespace std::chrono_literals;

    const arm::JointLimits limits{ { { foc::Radians{ -1.0f }, foc::Radians{ 1.0f } },
        { foc::Radians{ -2.0f }, foc::Radians{ 2.0f } },
        { foc::Radians{ -2.0f }, foc::Radians{ 0.5f } },
        { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } },
        { foc::Radians{ -1.5f }, foc::Radians{ 1.5f } },
        { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } } } };

    const arm::JointPositions withinLimits{ foc::Radians{ -1.0f }, foc::Radians{ 0.0f }, foc::Radians{ 0.5f },
        foc::Radians{ 1.0f }, foc::Radians{ -1.0f }, foc::Radians{ 3.0f } };

    arm::TrackingLimits Tracking()
    {
        arm::TrackingLimits tracking{};
        tracking.fill({ foc::Radians{ 0.2f }, 30ms });
        return tracking;
    }

    arm::CollisionLimits Collision()
    {
        arm::CollisionLimits collision{};
        collision.fill(foc::NewtonMeter{ 5.0f });
        return collision;
    }

    arm::MotionSupervision Motion()
    {
        arm::MotionSupervision motion{};
        motion.reference.fill({ foc::RadiansPerSecond{ 30.0f }, foc::RadiansPerSecondSquared{ 100.0f } });
        motion.speed.fill(foc::RadiansPerSecond{ 4.0f });
        motion.torque.fill(foc::NewtonMeter{ 20.0f });
        return motion;
    }

    arm::JointTorques Torques(float j1, float j2, float j3)
    {
        return { foc::NewtonMeter{ j1 }, foc::NewtonMeter{ j2 }, foc::NewtonMeter{ j3 }, foc::NewtonMeter{ 0.0f }, foc::NewtonMeter{ 0.0f }, foc::NewtonMeter{ 0.0f } };
    }

    class SafetySupervisorImplTest
        : public Test
    {
    public:
        SafetySupervisorImplTest()
        {
            EXPECT_CALL(guard, Check(_)).WillRepeatedly(Return(std::nullopt));
            EXPECT_CALL(hardwareStop, Open()).WillRepeatedly(Invoke([this]()
                {
                    return stopOpen;
                }));
        }

        void ChangeHardwareStop(bool open)
        {
            stopOpen = open;
            hardwareStop.NotifyObservers([open](auto& observer)
                {
                    observer.OnHardwareStopChanged(open);
                });
        }

        void CompatibleContracts()
        {
            EXPECT_CALL(bus, Contracts()).WillRepeatedly(Return(can::ContractCompatibility::compatible));
        }

        void Trip()
        {
            EXPECT_CALL(bus, EmergencyStop()).WillOnce(Return(true));
            EXPECT_CALL(observer, OnJointFault(4, can::FocFaultCode::overCurrent));
            bus.NotifyObservers([](auto& observer)
                {
                    observer.OnJointFault(4, can::FocFaultCode::overCurrent);
                });
        }

        void JointStates(can::FocMotorState faultedJointState)
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                readings[joint].state = joint == 4 ? faultedJointState : can::FocMotorState::idle;
                EXPECT_CALL(bus, Reading(joint)).WillRepeatedly(ReturnRef(readings[joint]));
            }
        }

        void Measured(const arm::JointPositions& positions, uint32_t period = 7)
        {
            EXPECT_CALL(bus, Period()).WillRepeatedly(Return(7));
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                readings[joint].position = positions[joint];
                readings[joint].period = period;
                EXPECT_CALL(bus, Reading(joint)).WillRepeatedly(ReturnRef(readings[joint]));
            }
        }

        void Hold()
        {
            auto beyond = withinLimits;
            beyond[2] = foc::Radians{ 0.55f };
            Measured(beyond);
            EXPECT_CALL(observer, OnProtectiveStop(2, arm::SupervisionTrip::positionLimit, 0.55f));
            supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);
        }

        StrictMock<arm::JointBusMock> bus;
        StrictMock<arm::PoseGuardMock> guard;
        StrictMock<arm::HardwareStopMock> hardwareStop;
        bool stopOpen{ false };
        arm::SafetySupervisorImpl supervisor{ bus, limits, Tracking(), Collision(), 10ms, Motion(), &guard, &hardwareStop };
        StrictMock<arm::JointBusObserverMock> observer{ supervisor };
        std::array<arm::JointReading, arm::jointCount> readings{};
    };
}

TEST_F(SafetySupervisorImplTest, passes_setpoints_within_the_limits_ends_included)
{
    CompatibleContracts();
    EXPECT_CALL(bus, SendPositions(withinLimits)).WillOnce(Return(true));

    EXPECT_TRUE(supervisor.SendPositions(withinLimits));
}

TEST_F(SafetySupervisorImplTest, refuses_a_setpoint_set_with_any_position_outside_its_joint_limits)
{
    CompatibleContracts();

    auto aboveUpper = withinLimits;
    aboveUpper[2] = foc::Radians{ 0.51f };
    auto belowLower = withinLimits;
    belowLower[0] = foc::Radians{ -1.01f };

    EXPECT_FALSE(supervisor.SendPositions(aboveUpper));
    EXPECT_FALSE(supervisor.SendPositions(belowLower));
}

TEST_F(SafetySupervisorImplTest, refuses_a_setpoint_set_with_a_position_that_is_not_finite)
{
    CompatibleContracts();

    auto notANumber = withinLimits;
    notANumber[5] = foc::Radians{ std::numeric_limits<float>::quiet_NaN() };

    EXPECT_FALSE(supervisor.SendPositions(notANumber));
}

TEST_F(SafetySupervisorImplTest, passes_a_finite_torque_set_on_and_refuses_one_that_is_not)
{
    CompatibleContracts();
    arm::JointTorques torques{};
    torques[1] = foc::NewtonMeter{ 12.0f };
    EXPECT_CALL(bus, SendTorques(torques)).WillOnce(Return(true));
    EXPECT_TRUE(supervisor.SendTorques(torques));

    torques[4] = foc::NewtonMeter{ std::numeric_limits<float>::infinity() };
    EXPECT_FALSE(supervisor.SendTorques(torques));
}

TEST_F(SafetySupervisorImplTest, a_torque_set_the_joint_bus_cannot_queue_in_full_is_a_missed_period_not_a_refusal)
{
    CompatibleContracts();
    arm::JointTorques torques{};
    torques[1] = foc::NewtonMeter{ 12.0f };

    EXPECT_CALL(bus, SendTorques(torques)).WillOnce(Return(false));
    EXPECT_TRUE(supervisor.SendTorques(torques));
}

TEST_F(SafetySupervisorImplTest, refuses_motion_until_every_contract_is_compatible)
{
    EXPECT_CALL(bus, Contracts()).WillRepeatedly(Return(can::ContractCompatibility::unknown));

    EXPECT_FALSE(supervisor.SelectMode(can::FocMotorMode::position));
    EXPECT_FALSE(supervisor.Start());
    EXPECT_FALSE(supervisor.Align());
    EXPECT_FALSE(supervisor.SendPositions(withinLimits));
    EXPECT_FALSE(supervisor.SendTorques({}));
}

TEST_F(SafetySupervisorImplTest, passes_motion_commands_on_once_contracts_are_compatible)
{
    CompatibleContracts();
    EXPECT_CALL(bus, SelectMode(can::FocMotorMode::position)).WillOnce(Return(true));
    EXPECT_CALL(bus, Start()).WillOnce(Return(true));
    EXPECT_CALL(bus, Align()).WillOnce(Return(true));

    EXPECT_TRUE(supervisor.SelectMode(can::FocMotorMode::position));
    EXPECT_TRUE(supervisor.Start());
    EXPECT_TRUE(supervisor.Align());
}

TEST_F(SafetySupervisorImplTest, always_passes_stopping_clearing_and_queries_on)
{
    InSequence order;
    EXPECT_CALL(bus, Stop()).WillOnce(Return(true));
    EXPECT_CALL(bus, EmergencyStop()).WillOnce(Return(false));
    EXPECT_CALL(bus, ClearFault()).WillOnce(Return(true));
    EXPECT_CALL(bus, QueryContracts()).WillOnce(Return(true));
    EXPECT_CALL(bus, RequestTelemetry()).WillOnce(Return(true));

    EXPECT_TRUE(supervisor.Stop());
    EXPECT_FALSE(supervisor.EmergencyStop());
    EXPECT_TRUE(supervisor.ClearFault());
    EXPECT_TRUE(supervisor.QueryContracts());
    EXPECT_TRUE(supervisor.RequestTelemetry());
}

TEST_F(SafetySupervisorImplTest, a_faulted_joint_stops_every_joint_and_is_passed_on)
{
    EXPECT_FALSE(supervisor.Tripped());

    Trip();

    EXPECT_TRUE(supervisor.Tripped());
}

TEST_F(SafetySupervisorImplTest, a_silent_joint_stops_every_joint_and_is_passed_on)
{
    EXPECT_CALL(bus, EmergencyStop()).WillOnce(Return(true));
    EXPECT_CALL(observer, OnJointSilent(2));

    bus.NotifyObservers([](auto& observer)
        {
            observer.OnJointSilent(2);
        });

    EXPECT_TRUE(supervisor.Tripped());
}

TEST_F(SafetySupervisorImplTest, a_second_event_while_tripped_is_passed_on_without_another_emergency_stop)
{
    Trip();

    EXPECT_CALL(observer, OnJointSilent(1));
    bus.NotifyObservers([](auto& observer)
        {
            observer.OnJointSilent(1);
        });
}

TEST_F(SafetySupervisorImplTest, refuses_motion_while_tripped)
{
    CompatibleContracts();
    Trip();

    EXPECT_FALSE(supervisor.SelectMode(can::FocMotorMode::position));
    EXPECT_FALSE(supervisor.Start());
    EXPECT_FALSE(supervisor.Align());
    EXPECT_FALSE(supervisor.SendPositions(withinLimits));
}

TEST_F(SafetySupervisorImplTest, refuses_to_rearm_while_a_joint_is_faulted)
{
    Trip();
    JointStates(can::FocMotorState::fault);

    EXPECT_FALSE(supervisor.Rearm());
    EXPECT_TRUE(supervisor.Tripped());
}

TEST_F(SafetySupervisorImplTest, rearms_once_no_joint_is_faulted_and_motion_passes_again)
{
    CompatibleContracts();
    Trip();
    JointStates(can::FocMotorState::idle);

    EXPECT_TRUE(supervisor.Rearm());

    EXPECT_FALSE(supervisor.Tripped());
    EXPECT_CALL(bus, Start()).WillOnce(Return(true));
    EXPECT_TRUE(supervisor.Start());
}

TEST_F(SafetySupervisorImplTest, passes_contract_outcomes_and_readings_on)
{
    EXPECT_CALL(observer, OnContractsResolved(can::ContractCompatibility::incompatible));
    bus.NotifyObservers([](auto& observer)
        {
            observer.OnContractsResolved(can::ContractCompatibility::incompatible);
        });

    JointStates(can::FocMotorState::running);
    EXPECT_EQ(&readings[4], &supervisor.Reading(4));
    EXPECT_CALL(bus, Contracts()).WillOnce(Return(can::ContractCompatibility::incompatible));
    EXPECT_EQ(can::ContractCompatibility::incompatible, supervisor.Contracts());
    EXPECT_CALL(bus, Period()).WillOnce(Return(17u));
    EXPECT_EQ(17u, supervisor.Period());
}

TEST_F(SafetySupervisorImplTest, passes_command_delivery_outcomes_on)
{
    EXPECT_CALL(observer, OnCommandDelivered(arm::JointCommand::start));
    bus.NotifyObservers([](auto& observer)
        {
            observer.OnCommandDelivered(arm::JointCommand::start);
        });

    EXPECT_CALL(observer, OnCommandFailed(arm::JointCommand::stop, arm::CommandFailure::timedOut));
    bus.NotifyObservers([](auto& observer)
        {
            observer.OnCommandFailed(arm::JointCommand::stop, arm::CommandFailure::timedOut);
        });
}

TEST_F(SafetySupervisorImplTest, passes_per_joint_configuration_through_and_its_answers_on)
{
    EXPECT_CALL(bus, SelectAlgorithm(2, can::FocControlLoop::current, 3)).WillOnce(Return(true));
    EXPECT_TRUE(supervisor.SelectAlgorithm(2, can::FocControlLoop::current, 3));
    EXPECT_CALL(bus, QueryAlgorithms(2)).WillOnce(Return(true));
    EXPECT_TRUE(supervisor.QueryAlgorithms(2));
    EXPECT_CALL(bus, SetParameter(2, can::FocParameter::currentBandwidth, 500.0f)).WillOnce(Return(false));
    EXPECT_FALSE(supervisor.SetParameter(2, can::FocParameter::currentBandwidth, 500.0f));
    EXPECT_CALL(bus, QueryParameter(2, can::FocParameter::currentBandwidth)).WillOnce(Return(true));
    EXPECT_TRUE(supervisor.QueryParameter(2, can::FocParameter::currentBandwidth));

    EXPECT_CALL(observer, OnAlgorithms(5, _));
    bus.NotifyObservers([](auto& joints)
        {
            joints.OnAlgorithms(5, can::FocAlgorithms{});
        });
    EXPECT_CALL(observer, OnParameter(5, can::FocParameter::speedBandwidth, can::FocParameterValue{ 80.0f, true }));
    bus.NotifyObservers([](auto& joints)
        {
            joints.OnParameter(5, can::FocParameter::speedBandwidth, can::FocParameterValue{ 80.0f, true });
        });
}

TEST_F(SafetySupervisorImplTest, a_measured_position_on_its_limits_keeps_the_arm_armed)
{
    Measured(withinLimits);

    supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);

    EXPECT_FALSE(supervisor.Tripped());
}

TEST_F(SafetySupervisorImplTest, a_measured_position_outside_its_limits_stops_the_arm_protectively)
{
    auto beyond = withinLimits;
    beyond[2] = foc::Radians{ 0.55f };
    Measured(beyond);

    EXPECT_CALL(observer, OnProtectiveStop(2, arm::SupervisionTrip::positionLimit, 0.55f));
    supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);

    EXPECT_TRUE(supervisor.Holding());
    EXPECT_FALSE(supervisor.Tripped());
}

TEST_F(SafetySupervisorImplTest, a_tracking_error_held_above_its_window_for_its_time_stops_the_arm_protectively)
{
    Measured(withinLimits);
    auto reference = withinLimits;
    reference[1] = foc::Radians{ 0.25f };

    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    EXPECT_FALSE(supervisor.Holding());

    EXPECT_CALL(observer, OnProtectiveStop(1, arm::SupervisionTrip::trackingError, 0.25f));
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);

    EXPECT_TRUE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, a_step_within_the_window_or_without_streaming_starts_the_count_again)
{
    Measured(withinLimits);
    auto reference = withinLimits;
    reference[1] = foc::Radians{ -0.25f };

    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    supervisor.Supervise(std::nullopt, std::nullopt);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);

    EXPECT_FALSE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, a_stale_reading_is_neither_checked_nor_counted_nor_starts_the_count_again)
{
    auto beyond = withinLimits;
    beyond[0] = foc::Radians{ -1.5f };
    Measured(beyond, 5);
    auto reference = withinLimits;
    reference[1] = foc::Radians{ 0.25f };

    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    EXPECT_FALSE(supervisor.Holding());

    Measured(withinLimits);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    Measured(withinLimits, 5);
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
    Measured(withinLimits);
    EXPECT_CALL(observer, OnProtectiveStop(1, arm::SupervisionTrip::trackingError, 0.25f));
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);
}

TEST_F(SafetySupervisorImplTest, without_streaming_a_position_outside_its_limits_is_not_checked)
{
    auto beyond = withinLimits;
    beyond[0] = foc::Radians{ -1.5f };
    Measured(beyond);

    supervisor.Supervise(std::nullopt, std::nullopt);

    EXPECT_FALSE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, a_tripped_arm_is_not_supervised_again)
{
    Trip();
    auto beyond = withinLimits;
    beyond[0] = foc::Radians{ -1.5f };
    Measured(beyond);

    supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);
}

TEST_F(SafetySupervisorImplTest, an_external_torque_within_its_joints_limit_keeps_the_arm_armed)
{
    Measured(withinLimits);
    arm::JointTorques external{};
    external[3] = foc::NewtonMeter{ -5.0f };

    supervisor.Supervise(arm::AtRest(withinLimits), external);

    EXPECT_FALSE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, an_external_torque_beyond_its_joints_limit_stops_the_arm_protectively_on_a_collision)
{
    Measured(withinLimits);
    arm::JointTorques external{};
    external[3] = foc::NewtonMeter{ -5.5f };

    EXPECT_CALL(observer, OnProtectiveStop(3, arm::SupervisionTrip::collision, -5.5f));
    supervisor.Supervise(arm::AtRest(withinLimits), external);

    EXPECT_TRUE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, without_streaming_an_external_torque_is_not_checked)
{
    arm::JointTorques external{};
    external[3] = foc::NewtonMeter{ 50.0f };

    supervisor.Supervise(std::nullopt, external);

    EXPECT_FALSE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, while_holding_torques_pass_on_and_mode_start_and_align_are_refused)
{
    CompatibleContracts();
    Hold();

    EXPECT_CALL(bus, SendTorques(_)).WillOnce(Return(true));
    EXPECT_TRUE(supervisor.SendTorques({}));
    EXPECT_FALSE(supervisor.SelectMode(can::FocMotorMode::torque));
    EXPECT_FALSE(supervisor.Start());
    EXPECT_FALSE(supervisor.Align());
}

TEST_F(SafetySupervisorImplTest, while_holding_neither_collision_nor_position_limits_are_checked_again)
{
    Hold();
    auto held = withinLimits;
    held[2] = foc::Radians{ 0.55f };
    arm::JointTorques external{};
    external[3] = foc::NewtonMeter{ 50.0f };

    supervisor.Supervise(arm::AtRest(held), external);
    supervisor.Supervise(arm::AtRest(held), external);
    supervisor.Supervise(arm::AtRest(held), external);

    EXPECT_TRUE(supervisor.Holding());
    EXPECT_FALSE(supervisor.Tripped());
}

TEST_F(SafetySupervisorImplTest, a_hold_that_drifts_out_of_its_window_for_its_time_emergency_stops_the_arm)
{
    Hold();
    auto held = withinLimits;
    held[2] = foc::Radians{ 0.8f };

    supervisor.Supervise(arm::AtRest(held), std::nullopt);
    supervisor.Supervise(arm::AtRest(held), std::nullopt);

    EXPECT_CALL(bus, EmergencyStop()).WillOnce(Return(true));
    EXPECT_CALL(observer, OnSupervisionTrip(2, arm::SupervisionTrip::holdLost, FloatNear(0.25f, 1e-5f)));
    supervisor.Supervise(arm::AtRest(held), std::nullopt);

    EXPECT_TRUE(supervisor.Tripped());
    EXPECT_FALSE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, a_joint_fault_while_holding_ends_the_hold_in_an_emergency_stop)
{
    Hold();

    Trip();

    EXPECT_TRUE(supervisor.Tripped());
    EXPECT_FALSE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, rearming_a_protective_stop_resumes_supervision_and_motion)
{
    CompatibleContracts();
    Hold();

    EXPECT_TRUE(supervisor.Rearm());
    EXPECT_FALSE(supervisor.Holding());

    EXPECT_CALL(bus, Start()).WillOnce(Return(true));
    EXPECT_TRUE(supervisor.Start());
    EXPECT_CALL(observer, OnProtectiveStop(2, arm::SupervisionTrip::positionLimit, 0.55f));
    supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);
    EXPECT_TRUE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, clamps_each_torque_to_its_joints_limit_and_reports_the_clamped_joints)
{
    CompatibleContracts();

    EXPECT_CALL(bus, SendTorques(Torques(20.0f, -20.0f, 3.0f))).WillOnce(Return(true));
    EXPECT_TRUE(supervisor.SendTorques(Torques(22.0f, -25.0f, 3.0f)));

    EXPECT_EQ((arm::JointFlags{ true, true, false, false, false, false }), supervisor.TorqueClamped());
}

TEST_F(SafetySupervisorImplTest, a_measured_speed_above_its_limit_stops_the_arm_protectively)
{
    Measured(withinLimits);
    readings[3].speed = foc::RadiansPerSecond{ -4.5f };

    EXPECT_CALL(observer, OnProtectiveStop(3, arm::SupervisionTrip::speedLimit, -4.5f));
    supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);

    EXPECT_TRUE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, a_reference_faster_than_its_joints_motion_limit_stops_the_arm_protectively)
{
    Measured(withinLimits);
    auto reference = arm::AtRest(withinLimits);
    reference.speeds[2] = foc::RadiansPerSecond{ 31.0f };

    EXPECT_CALL(observer, OnProtectiveStop(2, arm::SupervisionTrip::referenceSpeed, 31.0f));
    supervisor.Supervise(reference, std::nullopt);
}

TEST_F(SafetySupervisorImplTest, a_reference_accelerating_harder_than_its_joints_motion_limit_stops_the_arm_protectively)
{
    Measured(withinLimits);
    auto reference = arm::AtRest(withinLimits);
    reference.accelerations[4] = foc::RadiansPerSecondSquared{ -101.0f };

    EXPECT_CALL(observer, OnProtectiveStop(4, arm::SupervisionTrip::referenceAcceleration, -101.0f));
    supervisor.Supervise(reference, std::nullopt);
}

TEST_F(SafetySupervisorImplTest, a_reference_that_jumps_further_than_its_speed_and_a_step_allow_stops_the_arm_protectively)
{
    Measured(withinLimits);
    auto within = withinLimits;
    within[0] = foc::Radians{ -0.7f };
    auto jump = withinLimits;
    jump[0] = foc::Radians{ -0.35f };

    supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);
    supervisor.Supervise(arm::AtRest(within), std::nullopt);
    EXPECT_FALSE(supervisor.Holding());

    EXPECT_CALL(observer, OnProtectiveStop(0, arm::SupervisionTrip::referenceStep, FloatNear(0.35f, 1e-5f)));
    supervisor.Supervise(arm::AtRest(jump), std::nullopt);
}

TEST_F(SafetySupervisorImplTest, the_reference_jump_counts_from_the_last_streaming_step_only)
{
    Measured(withinLimits);
    auto jump = withinLimits;
    jump[0] = foc::Radians{ -0.6f };

    supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);
    supervisor.Supervise(std::nullopt, std::nullopt);
    supervisor.Supervise(arm::AtRest(jump), std::nullopt);

    EXPECT_FALSE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, a_reference_pose_the_workspace_check_fails_stops_the_arm_protectively_and_is_kept)
{
    Measured(withinLimits);
    auto reference = withinLimits;
    reference[1] = foc::Radians{ 0.1f };
    const arm::WorkspaceViolation violation{ arm::WorkspaceRule::forbiddenZone, 4, 1, 0.02f };
    EXPECT_CALL(guard, Check(reference)).WillOnce(Return(violation));

    EXPECT_CALL(observer, OnProtectiveStop(4, arm::SupervisionTrip::forbiddenZone, 0.02f));
    supervisor.Supervise(arm::AtRest(reference), std::nullopt);

    EXPECT_EQ(std::optional{ violation }, supervisor.LastWorkspaceViolation());
}

TEST_F(SafetySupervisorImplTest, a_measured_pose_the_workspace_check_fails_stops_the_arm_protectively)
{
    auto measured = withinLimits;
    measured[2] = foc::Radians{ 0.4f };
    Measured(measured);
    const arm::WorkspaceViolation violation{ arm::WorkspaceRule::selfCollision, 0, 2, 0.001f };
    EXPECT_CALL(guard, Check(measured)).WillOnce(Return(violation));

    EXPECT_CALL(observer, OnProtectiveStop(0, arm::SupervisionTrip::selfCollision, 0.001f));
    supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);
}

TEST_F(SafetySupervisorImplTest, a_measured_pose_with_a_stale_joint_is_not_checked_against_the_workspace)
{
    auto measured = withinLimits;
    measured[2] = foc::Radians{ 0.4f };
    Measured(measured, 5);
    EXPECT_CALL(guard, Check(measured)).Times(0);

    supervisor.Supervise(arm::AtRest(withinLimits), std::nullopt);

    EXPECT_FALSE(supervisor.Holding());
}

TEST_F(SafetySupervisorImplTest, the_hardware_stop_opening_emergency_stops_every_joint_and_is_passed_on)
{
    CompatibleContracts();

    EXPECT_CALL(bus, EmergencyStop()).WillOnce(Return(true));
    EXPECT_CALL(observer, OnHardwareStop(true));
    ChangeHardwareStop(true);

    EXPECT_TRUE(supervisor.Tripped());
    EXPECT_TRUE(supervisor.HardwareStopOpen());
    EXPECT_FALSE(supervisor.Start());
    EXPECT_FALSE(supervisor.SendTorques({}));
}

TEST_F(SafetySupervisorImplTest, refuses_to_rearm_while_the_hardware_stop_is_open)
{
    JointStates(can::FocMotorState::ready);
    EXPECT_CALL(bus, EmergencyStop()).WillOnce(Return(true));
    EXPECT_CALL(observer, OnHardwareStop(true));
    ChangeHardwareStop(true);

    EXPECT_FALSE(supervisor.Rearm());

    EXPECT_CALL(observer, OnHardwareStop(false));
    ChangeHardwareStop(false);
    EXPECT_TRUE(supervisor.Tripped());
    EXPECT_TRUE(supervisor.Rearm());
}

TEST_F(SafetySupervisorImplTest, a_hardware_stop_open_from_the_start_refuses_motion_without_a_trip)
{
    CompatibleContracts();
    stopOpen = true;

    EXPECT_FALSE(supervisor.Start());
    EXPECT_FALSE(supervisor.SelectMode(can::FocMotorMode::torque));
    EXPECT_FALSE(supervisor.Tripped());
}
