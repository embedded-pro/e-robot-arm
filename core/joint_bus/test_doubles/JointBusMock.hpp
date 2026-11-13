#ifndef CORE_JOINT_BUS_TEST_DOUBLES_JOINT_BUS_MOCK_HPP
#define CORE_JOINT_BUS_TEST_DOUBLES_JOINT_BUS_MOCK_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class JointBusMock
        : public JointBus
    {
    public:
        MOCK_METHOD(bool, QueryContracts, (), (override));
        MOCK_METHOD(can::ContractCompatibility, Contracts, (), (const, override));
        MOCK_METHOD(bool, SelectMode, (can::FocMotorMode mode), (override));
        MOCK_METHOD(bool, Start, (), (override));
        MOCK_METHOD(bool, Stop, (), (override));
        MOCK_METHOD(bool, ClearFault, (), (override));
        MOCK_METHOD(bool, EmergencyStop, (), (override));
        MOCK_METHOD(bool, SendPositions, (const JointPositions& positions), (override));
        MOCK_METHOD(bool, RequestTelemetry, (), (override));
        MOCK_METHOD(bool, SelectAlgorithm, (std::size_t joint, can::FocControlLoop loop, uint8_t algorithm), (override));
        MOCK_METHOD(bool, QueryAlgorithms, (std::size_t joint), (override));
        MOCK_METHOD(bool, SetParameter, (std::size_t joint, can::FocParameter parameter, float value), (override));
        MOCK_METHOD(bool, QueryParameter, (std::size_t joint, can::FocParameter parameter), (override));
        MOCK_METHOD(const JointReading&, Reading, (std::size_t joint), (const, override));
        MOCK_METHOD(uint32_t, Period, (), (const, override));
    };

    class JointBusObserverMock
        : public JointBusObserver
    {
    public:
        using JointBusObserver::JointBusObserver;

        MOCK_METHOD(void, OnContractsResolved, (can::ContractCompatibility compatibility), (override));
        MOCK_METHOD(void, OnJointFault, (std::size_t joint, can::FocFaultCode fault), (override));
        MOCK_METHOD(void, OnJointSilent, (std::size_t joint), (override));
        MOCK_METHOD(void, OnCommandDelivered, (arm::JointCommand command), (override));
        MOCK_METHOD(void, OnCommandFailed, (arm::JointCommand command, arm::CommandFailure failure), (override));
        MOCK_METHOD(void, OnAlgorithms, (std::size_t joint, const can::FocAlgorithms& algorithms), (override));
        MOCK_METHOD(void, OnParameter, (std::size_t joint, can::FocParameter parameter, can::FocParameterValue value), (override));
    };
}

#endif
