#pragma once

#include "core/motion/interfaces/JointMotion.hpp"
#include "numerical/motion_profiles/SCurveProfile.hpp"
#include "numerical/motion_profiles/SCurveProfileFromState.hpp"
#include "numerical/motion_profiles/TrapezoidalProfile.hpp"
#include "numerical/motion_profiles/TrapezoidalProfileFromState.hpp"
#include <array>
#include <variant>

namespace arm
{
    class JointMovePlan
    {
    public:
        void Plan(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits, const JointJerkLimits& jerks = {},
            MotionProfile profile = MotionProfile::trapezoid);
        void Replan(const JointReference& from, const JointPositions& to, const JointMotionLimits& limits, const JointJerkLimits& jerks = {});
        JointReference Sample(float time) const;
        float Duration() const;

    private:
        void PlanTrapezoids(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits);
        void PlanSCurves(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits, const JointJerkLimits& jerks);

        using JointProfile = std::variant<std::monostate, motion_profiles::TrapezoidalProfile<float>, motion_profiles::SCurveProfile<float>,
            motion_profiles::TrapezoidalProfileFromState<float>, motion_profiles::SCurveProfileFromState<float>>;

        MotionProfile profile{ MotionProfile::trapezoid };
        std::array<JointProfile, jointCount> joints;
        float duration{ 0.0f };
    };
}
