#include "core/motion/implementations/JointDifferences.hpp"
#include "core/arm_model/JointFrameMap.hpp"
#include "numerical/math/Math.hpp"

namespace arm
{
    JointRates BackwardDifferences(const JointPositions& previous, const JointSpeeds& previousSpeeds, const JointPositions& next, float period)
    {
        const auto perPeriod = 1.0f / period;
        const JointVector speeds{ (ToVector(next) - ToVector(previous)) * perPeriod };
        const JointVector accelerations{ (speeds - ToVector(previousSpeeds)) * perPeriod };
        return { FromVector<foc::RadiansPerSecond>(speeds), FromVector<foc::RadiansPerSecondSquared>(accelerations) };
    }

    bool WithinMotionLimits(const JointRates& rates, const JointMotionLimits& limits)
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (math::Abs(rates.speeds[joint].Value()) > limits[joint].maxSpeed.Value() ||
                math::Abs(rates.accelerations[joint].Value()) > limits[joint].maxAcceleration.Value())
                return false;
        return true;
    }
}
