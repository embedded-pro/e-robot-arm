#ifndef CORE_MOTION_IMPLEMENTATIONS_JOINT_PATH_PLAN_HPP
#define CORE_MOTION_IMPLEMENTATIONS_JOINT_PATH_PLAN_HPP

#include "core/motion/interfaces/JointMotion.hpp"
#include "robotics/trajectory/CubicSplineTrajectory.hpp"
#include "robotics/trajectory/TimeOptimalPathParameterization.hpp"

namespace arm
{
    constexpr std::size_t pathGrid{ 64 };
    constexpr std::size_t maxSlowdownRounds{ 8 };

    class JointPathPlan
        : private trajectory::PathGeometry<float, jointCount>
        , private trajectory::PathConstraints<float, jointCount>
    {
    public:
        JointPathPlan();
        JointPathPlan(const JointPathPlan& other) = delete;
        JointPathPlan& operator=(const JointPathPlan& other) = delete;

        bool Plan(const JointPositions& from, const JointWaypoints& waypoints, const JointMotionLimits& limits, float samplePeriod);
        JointReference Sample(float time) const;
        float Duration() const;

    private:
        using JointVector = math::Vector<float, jointCount>;
        using Spline = trajectory::CubicSplineTrajectory<float, jointCount, maxWaypoints + 1>;

        bool Fit(const JointPositions& from, const JointWaypoints& waypoints);
        bool SlowToLimits(const JointMotionLimits& limits, float samplePeriod);
        float Excess(const JointMotionLimits& limits, float samplePeriod) const;

        JointVector Position(float s) const override;
        JointVector FirstDerivative(float s) const override;
        JointVector SecondDerivative(float s) const override;
        trajectory::PathCoefficients<float, jointCount> Coefficients(float s) const override;
        const trajectory::JointBounds<float, jointCount>& Bounds() const override;

        Spline spline;
        trajectory::JointBounds<float, jointCount> bounds;
        trajectory::TimeOptimalPathParameterization<float, jointCount, pathGrid> timing;
        float slowdown{ 1.0f };
    };
}

#endif
