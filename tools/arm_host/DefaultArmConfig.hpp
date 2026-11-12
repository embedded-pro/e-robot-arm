#ifndef TOOLS_ARM_HOST_DEFAULT_ARM_CONFIG_HPP
#define TOOLS_ARM_HOST_DEFAULT_ARM_CONFIG_HPP

#include "core/arm/Arm.hpp"
#include "infra/timer/Timer.hpp"

namespace arm_host
{
    inline constexpr arm::JointNodeIds defaultNodes{ 1, 2, 3, 4, 5, 6 };
    inline constexpr infra::Duration simulatedTimeConstant{ std::chrono::milliseconds{ 50 } };

    arm::ArmConfig DefaultArmConfig(const arm::JointNodeIds& nodes);
}

#endif
