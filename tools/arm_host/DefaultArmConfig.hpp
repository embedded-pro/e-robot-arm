#pragma once

#include "core/arm/Arm.hpp"
#include "core/arm_profile/ArmProfile.hpp"
#include "infra/timer/Timer.hpp"

namespace arm_host
{
    inline constexpr arm::JointNodeIds defaultNodes{ 1, 2, 3, 4, 5, 6 };
    inline constexpr infra::Duration simulatedTimeConstant{ std::chrono::milliseconds{ 50 } };

    arm::ArmConfig DefaultArmConfig(const arm::JointNodeIds& nodes);
    arm::ArmProfile DefaultArmProfile(const arm::JointNodeIds& nodes);
}
