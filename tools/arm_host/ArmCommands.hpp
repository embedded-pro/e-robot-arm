#ifndef TOOLS_ARM_HOST_ARM_COMMANDS_HPP
#define TOOLS_ARM_HOST_ARM_COMMANDS_HPP

#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include <cstdint>

namespace arm_host
{
    class ArmCommands
    {
    public:
        virtual uint32_t Post(const ArmCommand& command) = 0;
        virtual uint32_t SwitchTarget(const ArmTarget& target) = 0;
        virtual ArmSnapshot Snapshot() const = 0;

    protected:
        ~ArmCommands() = default;
    };
}

#endif
