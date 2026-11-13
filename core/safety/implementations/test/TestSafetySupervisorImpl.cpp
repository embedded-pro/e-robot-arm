#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "core/safety/implementations/SafetySupervisorImpl.hpp"
#include "gmock/gmock.h"
#include <limits>

namespace
{
    using namespace testing;

    const arm::JointLimits limits{ { { foc::Radians{ -1.0f }, foc::Radians{ 1.0f } },
        { foc::Radians{ -2.0f }, foc::Radians{ 2.0f } },
        { foc::Radians{ -2.0f }, foc::Radians{ 0.5f } },
        { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } },
        { foc::Radians{ -1.5f }, foc::Radians{ 1.5f } },
        { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } } } };

    const arm::JointPositions withinLimits{ foc::Radians{ -1.0f }, foc::Radians{ 0.0f }, foc::Radians{ 0.5f },
        foc::Radians{ 1.0f }, foc::Radians{ -1.0f }, foc::Radians{ 3.0f } };

    class SafetySupervisorImplTest
        : public Test
    {
    public:
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

        StrictMock<arm::JointBusMock> bus;
        arm::SafetySupervisorImpl supervisor{ bus, limits };
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

TEST_F(SafetySupervisorImplTest, refuses_motion_until_every_contract_is_compatible)
{
    EXPECT_CALL(bus, Contracts()).WillRepeatedly(Return(can::ContractCompatibility::unknown));

    EXPECT_FALSE(supervisor.SelectMode(can::FocMotorMode::position));
    EXPECT_FALSE(supervisor.Start());
    EXPECT_FALSE(supervisor.SendPositions(withinLimits));
}

TEST_F(SafetySupervisorImplTest, passes_motion_commands_on_once_contracts_are_compatible)
{
    CompatibleContracts();
    EXPECT_CALL(bus, SelectMode(can::FocMotorMode::position)).WillOnce(Return(true));
    EXPECT_CALL(bus, Start()).WillOnce(Return(true));

    EXPECT_TRUE(supervisor.SelectMode(can::FocMotorMode::position));
    EXPECT_TRUE(supervisor.Start());
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
