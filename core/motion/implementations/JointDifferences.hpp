#pragma once

#include "core/motion/interfaces/JointMotion.hpp"

namespace arm
{
    struct JointRates
    {
        JointSpeeds speeds{};
        JointAccelerations accelerations{};
    };

    JointRates BackwardDifferences(const JointPositions& previous, const JointSpeeds& previousSpeeds, const JointPositions& next, float period);
    bool WithinMotionLimits(const JointRates& rates, const JointMotionLimits& limits);
}
