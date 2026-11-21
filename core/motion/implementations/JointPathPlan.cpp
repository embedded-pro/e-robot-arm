#include "core/motion/implementations/JointPathPlan.hpp"
#include "core/arm_model/JointFrameMap.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include <algorithm>

namespace arm
{
    namespace
    {
        float Chord(const JointPositions& from, const JointPositions& to)
        {
            return math::VectorNorm(JointVector{ ToVector(to) - ToVector(from) });
        }
    }

    JointPathPlan::JointPathPlan()
        : timing(*this, *this)
    {}

    bool JointPathPlan::Plan(const JointPositions& from, const JointWaypoints& waypoints, const JointMotionLimits& limits, float samplePeriod)
    {
        slowdown = 1.0f;
        if (waypoints.empty() || waypoints.size() > maxWaypoints || !Fit(from, waypoints))
            return false;

        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            bounds.velocityMax[joint] = limits[joint].maxSpeed.Value();
            bounds.torqueMax[joint] = limits[joint].maxAcceleration.Value();
            bounds.torqueMin[joint] = -limits[joint].maxAcceleration.Value();
        }
        return timing.Parameterize() && SlowToLimits(limits, samplePeriod);
    }

    bool JointPathPlan::SlowToLimits(const JointMotionLimits& limits, float samplePeriod)
    {
        for (std::size_t round = 0; round != maxSlowdownRounds; ++round)
        {
            const auto excess = Excess(limits, samplePeriod);
            if (excess <= 1.0f)
                return true;
            slowdown *= excess;
        }
        return false;
    }

    float JointPathPlan::Excess(const JointMotionLimits& limits, float samplePeriod) const
    {
        float excess{ 0.0f };
        for (std::size_t index = 1; static_cast<float>(index) * samplePeriod < Duration(); ++index)
        {
            const auto sample = Sample(static_cast<float>(index) * samplePeriod);
            for (std::size_t joint = 0; joint != jointCount; ++joint)
            {
                excess = std::max(excess, math::Abs(sample.speeds[joint].Value()) / limits[joint].maxSpeed.Value());
                excess = std::max(excess, math::Sqrt(math::Abs(sample.accelerations[joint].Value()) / limits[joint].maxAcceleration.Value()));
            }
        }
        return excess > 1.0f ? excess * 1.001f : excess;
    }

    bool JointPathPlan::Fit(const JointPositions& from, const JointWaypoints& waypoints)
    {
        Spline::Times knots{};
        Spline::Positions points{};
        points[0] = ToVector(from);
        float length{ 0.0f };
        for (std::size_t index = 0; index != waypoints.size(); ++index)
        {
            const auto chord = Chord(index == 0 ? from : waypoints[index - 1], waypoints[index]);
            if (chord <= 0.0f)
                return false;
            length += chord;
            knots[index + 1] = length;
            points[index + 1] = ToVector(waypoints[index]);
        }

        for (std::size_t index = 1; index <= waypoints.size(); ++index)
            knots[index] /= length;
        return spline.Plan(knots, points, waypoints.size() + 1, Spline::Boundary{ math::SplineEnd::Natural });
    }

    JointReference JointPathPlan::Sample(float time) const
    {
        JointReference reference{};
        const auto state = timing.Sample(time / slowdown);
        if (!state.has_value())
            return reference;

        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            reference.positions[joint] = foc::Radians{ state->position[joint] };
            reference.speeds[joint] = foc::RadiansPerSecond{ state->velocity[joint] / slowdown };
            reference.accelerations[joint] = foc::RadiansPerSecondSquared{ state->acceleration[joint] / (slowdown * slowdown) };
        }
        return reference;
    }

    float JointPathPlan::Duration() const
    {
        return timing.TotalTime() * slowdown;
    }

    JointPathPlan::JointVector JointPathPlan::Position(float s) const
    {
        return spline.Sample(s).position;
    }

    JointPathPlan::JointVector JointPathPlan::FirstDerivative(float s) const
    {
        return spline.Sample(s).velocity;
    }

    JointPathPlan::JointVector JointPathPlan::SecondDerivative(float s) const
    {
        return spline.Sample(s).acceleration;
    }

    trajectory::PathCoefficients<float, jointCount> JointPathPlan::Coefficients(float s) const
    {
        const auto shape = spline.Sample(s);
        return { shape.velocity, shape.acceleration, JointVector{} };
    }

    const trajectory::JointBounds<float, jointCount>& JointPathPlan::Bounds() const
    {
        return bounds;
    }
}
