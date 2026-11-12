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
        MOCK_METHOD(const JointReading&, Reading, (std::size_t joint), (const, override));
    };

    class JointBusObserverMock
        : public JointBusObserver
    {
    public:
        using JointBusObserver::JointBusObserver;

        MOCK_METHOD(void, OnContractsResolved, (can::ContractCompatibility compatibility), (override));
        MOCK_METHOD(void, OnJointFault, (std::size_t joint, can::FocFaultCode fault), (override));
        MOCK_METHOD(void, OnJointSilent, (std::size_t joint), (override));
    };
}

#endif
