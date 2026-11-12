#ifndef TOOLS_ARM_HOST_ARM_TARGET_HPP
#define TOOLS_ARM_HOST_ARM_TARGET_HPP

#include <cstdint>
#include <string>

namespace arm_host
{
    enum class TargetKind : uint8_t
    {
        simulated,
        virtualBus,
        real
    };

    struct ArmTarget
    {
        TargetKind kind{ TargetKind::simulated };
        std::string bus;

        friend bool operator==(const ArmTarget& lhs, const ArmTarget& rhs) = default;
    };
}

#endif
