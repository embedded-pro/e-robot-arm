#include "core/motion/implementations/JointMovePlan.hpp"
#include <algorithm>

namespace arm
{
    void JointMovePlan::Plan(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits)
    {
        duration = 0.0f;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const motion_profiles::MotionLimits<float> jointLimits{ limits[joint].maxSpeed.Value(), limits[joint].maxAcceleration.Value(), 0.0f };
            profiles[joint].emplace(from[joint].Value(), to[joint].Value(), jointLimits);
            duration = std::max(duration, profiles[joint]->Duration());
        }

        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (auto stretched = motion_profiles::TrapezoidalProfile<float>::PlanWithDuration(from[joint].Value(), to[joint].Value(), limits[joint].maxAcceleration.Value(), duration))
                profiles[joint] = *stretched;
    }

    JointReference JointMovePlan::Sample(float time) const
    {
        JointReference reference{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto state = profiles[joint]->Sample(time);
            reference.positions[joint] = foc::Radians{ state.position };
            reference.speeds[joint] = foc::RadiansPerSecond{ state.velocity };
            reference.accelerations[joint] = foc::RadiansPerSecondSquared{ state.acceleration };
        }
        return reference;
    }

    float JointMovePlan::Duration() const
    {
        return duration;
    }
}
