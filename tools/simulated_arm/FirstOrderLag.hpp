#ifndef TOOLS_SIMULATED_ARM_FIRST_ORDER_LAG_HPP
#define TOOLS_SIMULATED_ARM_FIRST_ORDER_LAG_HPP

#include "e-foc-protocol/foc/Units.hpp"
#include "infra/timer/Timer.hpp"

namespace simulation
{
    struct JointMotionState
    {
        foc::Radians position;
        foc::RadiansPerSecond speed;
    };

    JointMotionState FollowWithLag(foc::Radians position, foc::Radians target, infra::Duration elapsed, infra::Duration timeConstant);
}

#endif
