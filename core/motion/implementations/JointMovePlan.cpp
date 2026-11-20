#include "core/motion/implementations/JointMovePlan.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>

namespace arm
{
    void JointMovePlan::Plan(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits, const JointJerkLimits& jerks, MotionProfile newProfile)
    {
        profile = newProfile;
        duration = 0.0f;
        if (profile == MotionProfile::sCurve)
            PlanSCurves(from, to, limits, jerks);
        else
            PlanTrapezoids(from, to, limits);
    }

    void JointMovePlan::PlanTrapezoids(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits)
    {
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

    void JointMovePlan::PlanSCurves(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits, const JointJerkLimits& jerks)
    {
        std::array<motion_profiles::MotionLimits<float>, jointCount> jointLimits{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            really_assert(jerks[joint].has_value());
            jointLimits[joint] = { limits[joint].maxSpeed.Value(), limits[joint].maxAcceleration.Value(), jerks[joint]->Value() };
            curves[joint].emplace(from[joint].Value(), to[joint].Value(), jointLimits[joint]);
            duration = std::max(duration, curves[joint]->Duration());
        }

        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto alone = curves[joint]->Duration();
            if (alone <= 0.0f || alone >= duration)
                continue;

            const auto scale = alone / duration;
            const auto& own = jointLimits[joint];
            curves[joint].emplace(from[joint].Value(), to[joint].Value(), motion_profiles::MotionLimits<float>{ own.vMax * scale, own.aMax * scale * scale, own.jMax * scale * scale * scale });
        }
    }

    JointReference JointMovePlan::Sample(float time) const
    {
        JointReference reference{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto state = profile == MotionProfile::sCurve ? curves[joint]->Sample(time) : profiles[joint]->Sample(time);
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
