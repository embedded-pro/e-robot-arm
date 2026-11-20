#ifndef CORE_MOTION_IMPLEMENTATIONS_JOINT_MOVE_PLAN_HPP
#define CORE_MOTION_IMPLEMENTATIONS_JOINT_MOVE_PLAN_HPP

#include "core/motion/interfaces/JointMotion.hpp"
#include "numerical/motion_profiles/SCurveProfile.hpp"
#include "numerical/motion_profiles/TrapezoidalProfile.hpp"
#include <array>
#include <optional>

namespace arm
{
    class JointMovePlan
    {
    public:
        void Plan(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits, const JointJerkLimits& jerks = {},
            MotionProfile profile = MotionProfile::trapezoid);
        JointReference Sample(float time) const;
        float Duration() const;

    private:
        void PlanTrapezoids(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits);
        void PlanSCurves(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits, const JointJerkLimits& jerks);

        MotionProfile profile{ MotionProfile::trapezoid };
        std::array<std::optional<motion_profiles::TrapezoidalProfile<float>>, jointCount> profiles;
        std::array<std::optional<motion_profiles::SCurveProfile<float>>, jointCount> curves;
        float duration{ 0.0f };
    };
}

#endif
