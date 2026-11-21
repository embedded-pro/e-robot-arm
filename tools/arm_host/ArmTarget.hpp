#pragma once

#include <cstdint>
#include <string>

namespace arm_host
{
    enum class TargetKind : uint8_t
    {
        simulated,
        virtualBus,
        real,
        virtualArm
    };

    struct ArmTarget
    {
        TargetKind kind{ TargetKind::simulated };
        std::string bus;
        std::string engine{};

        friend bool operator==(const ArmTarget& lhs, const ArmTarget& rhs) = default;
    };
}
