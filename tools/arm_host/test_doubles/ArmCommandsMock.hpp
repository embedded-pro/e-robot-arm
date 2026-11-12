#ifndef TOOLS_ARM_HOST_TEST_DOUBLES_ARM_COMMANDS_MOCK_HPP
#define TOOLS_ARM_HOST_TEST_DOUBLES_ARM_COMMANDS_MOCK_HPP

#include "tools/arm_host/ArmCommands.hpp"
#include "gmock/gmock.h"

namespace arm_host
{
    class ArmCommandsMock
        : public ArmCommands
    {
    public:
        MOCK_METHOD(uint32_t, Post, (const ArmCommand& command), (override));
        MOCK_METHOD(uint32_t, SwitchTarget, (const ArmTarget& target), (override));
        MOCK_METHOD(ArmSnapshot, Snapshot, (), (const, override));
    };
}

#endif
