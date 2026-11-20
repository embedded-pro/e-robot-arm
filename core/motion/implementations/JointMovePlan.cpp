#include "core/motion/implementations/JointMovePlan.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>

namespace arm
{
    namespace
    {
        struct SampleAt
        {
            motion_profiles::TrajectoryState<float> operator()(std::monostate) const
            {
                return {};
            }

            template<class Profile>
            motion_profiles::TrajectoryState<float> operator()(const Profile& profile) const
            {
                return profile.Sample(time);
            }

            float time;
        };
    }

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
            const auto& trapezoid = joints[joint].emplace<motion_profiles::TrapezoidalProfile<float>>(from[joint].Value(), to[joint].Value(), jointLimits);
            duration = std::max(duration, trapezoid.Duration());
        }

        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (auto stretched = motion_profiles::TrapezoidalProfile<float>::PlanWithDuration(from[joint].Value(), to[joint].Value(), limits[joint].maxAcceleration.Value(), duration))
                joints[joint] = *stretched;
    }

    void JointMovePlan::PlanSCurves(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits, const JointJerkLimits& jerks)
    {
        std::array<motion_profiles::MotionLimits<float>, jointCount> jointLimits{};
        std::array<float, jointCount> alone{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            really_assert(jerks[joint].has_value());
            jointLimits[joint] = { limits[joint].maxSpeed.Value(), limits[joint].maxAcceleration.Value(), jerks[joint]->Value() };
            alone[joint] = joints[joint].emplace<motion_profiles::SCurveProfile<float>>(from[joint].Value(), to[joint].Value(), jointLimits[joint]).Duration();
            duration = std::max(duration, alone[joint]);
        }

        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            if (alone[joint] <= 0.0f || alone[joint] >= duration)
                continue;

            const auto scale = alone[joint] / duration;
            const auto& own = jointLimits[joint];
            joints[joint].emplace<motion_profiles::SCurveProfile<float>>(from[joint].Value(), to[joint].Value(),
                motion_profiles::MotionLimits<float>{ own.vMax * scale, own.aMax * scale * scale, own.jMax * scale * scale * scale });
        }
    }

    void JointMovePlan::Replan(const JointReference& from, const JointPositions& to, const JointMotionLimits& limits, const JointJerkLimits& jerks)
    {
        duration = 0.0f;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto speed = limits[joint].maxSpeed.Value();
            const auto acceleration = limits[joint].maxAcceleration.Value();
            const auto position = from.positions[joint].Value();
            const auto velocity = from.speeds[joint].Value();
            float alone{};
            if (profile == MotionProfile::sCurve)
            {
                really_assert(jerks[joint].has_value());
                const auto running = std::clamp(from.accelerations[joint].Value(), -acceleration, acceleration);
                alone = joints[joint].emplace<motion_profiles::SCurveProfileFromState<float>>(position, velocity, running, to[joint].Value(),
                                         motion_profiles::JerkLimitedMotionLimits<float>{ speed, acceleration, acceleration, jerks[joint]->Value() })
                            .Duration();
            }
            else
                alone = joints[joint].emplace<motion_profiles::TrapezoidalProfileFromState<float>>(position, velocity, to[joint].Value(),
                                         motion_profiles::AsymmetricMotionLimits<float>{ speed, acceleration, acceleration })
                            .Duration();
            duration = std::max(duration, alone);
        }
    }

    JointReference JointMovePlan::Sample(float time) const
    {
        JointReference reference{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto state = std::visit(SampleAt{ time }, joints[joint]);
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
