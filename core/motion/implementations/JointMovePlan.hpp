#ifndef CORE_MOTION_IMPLEMENTATIONS_JOINT_MOVE_PLAN_HPP
#define CORE_MOTION_IMPLEMENTATIONS_JOINT_MOVE_PLAN_HPP

#include "core/motion/interfaces/JointMotion.hpp"
#include "numerical/motion_profiles/TrapezoidalProfile.hpp"
#include <array>
#include <optional>

namespace arm
{
    class JointMovePlan
    {
    public:
        void Plan(const JointPositions& from, const JointPositions& to, const JointMotionLimits& limits);
        JointPositions Sample(float time) const;
        float Duration() const;

    private:
        std::array<std::optional<motion_profiles::TrapezoidalProfile<float>>, jointCount> profiles;
        float duration{ 0.0f };
    };
}

#endif
