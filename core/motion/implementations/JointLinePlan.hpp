#ifndef CORE_MOTION_IMPLEMENTATIONS_JOINT_LINE_PLAN_HPP
#define CORE_MOTION_IMPLEMENTATIONS_JOINT_LINE_PLAN_HPP

#include "core/arm_model/ArmModel.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "core/safety/interfaces/PoseGuard.hpp"
#include "numerical/motion_profiles/SCurveProfile.hpp"
#include "robotics/trajectory/CartesianSlerpInterpolation.hpp"
#include <cstddef>
#include <optional>

namespace arm
{
    struct LineSample
    {
        std::optional<JointReference> reference;
        LineFault fault{ LineFault::unreachable };
    };

    class JointLinePlan
    {
    public:
        JointLinePlan(const ArmModel& model, const ToolLimits& tool, const JointMotionLimits& joints, const PoseGuard* guard, float period);

        std::optional<LineRefusal> Plan(const JointPositions& from, const ToolPose& target);
        LineSample Advance();
        bool Done() const;

    private:
        using Line = trajectory::CartesianSlerpInterpolation<float, motion_profiles::SCurveProfile<float>>;

        void Build(const ToolPose& start, const ToolPose& target);
        void Restart(const JointPositions& from);
        std::optional<LineFault> Check(const JointReference& reference) const;
        float Time(std::size_t index) const;

        const ArmModel& model;
        ToolLimits tool;
        JointMotionLimits joints;
        const PoseGuard* guard;
        float period;
        std::optional<Line> line;
        std::size_t periods{ 0 };
        std::size_t index{ 0 };
        JointPositions previous{};
        JointSpeeds previousSpeeds{};
    };
}

#endif
