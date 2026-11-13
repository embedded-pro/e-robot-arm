#ifndef CORE_SAFETY_TEST_DOUBLES_SAFETY_SUPERVISOR_MOCK_HPP
#define CORE_SAFETY_TEST_DOUBLES_SAFETY_SUPERVISOR_MOCK_HPP

#include "core/safety/interfaces/SafetySupervisor.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class SafetySupervisorMock
        : public SafetySupervisor
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
        MOCK_METHOD(bool, Tripped, (), (const, override));
        MOCK_METHOD(bool, Rearm, (), (override));
    };
}

#endif
