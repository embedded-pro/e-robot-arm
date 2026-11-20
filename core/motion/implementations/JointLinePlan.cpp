#include "core/motion/implementations/JointLinePlan.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Quaternion.hpp"
#include <algorithm>

namespace arm
{
    namespace
    {
        constexpr float shortest{ 1.0e-6f };

        float Length(const math::Vector3<float>& vector)
        {
            return math::Sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
        }

        float RotationAngle(const ToolPose& start, const ToolPose& target)
        {
            const auto from = math::Quaternion<float>::FromRotationMatrix(start.rotation);
            const auto to = math::Quaternion<float>::FromRotationMatrix(target.rotation);
            const auto dot = math::Abs(from.w * to.w + from.x * to.x + from.y * to.y + from.z * to.z);
            return 2.0f * math::Acos(std::min(dot, 1.0f));
        }

        motion_profiles::SCurveProfile<float> TimeLaw(float length, float speed, float acceleration, float jerk)
        {
            return motion_profiles::SCurveProfile<float>{ 0.0f, 1.0f, { speed / length, acceleration / length, jerk / length } };
        }
    }

    JointLinePlan::JointLinePlan(const ArmModel& model, const ToolLimits& tool, const JointMotionLimits& joints, const PoseGuard* guard, float period)
        : model(model)
        , tool(tool)
        , joints(joints)
        , guard(guard)
        , period(period)
    {}

    std::optional<LineRefusal> JointLinePlan::Plan(const JointPositions& from, const ToolPose& target)
    {
        Build(model.ToolPose(from), target);
        Restart(from);
        while (!Done())
        {
            const auto sample = Advance();
            if (!sample.reference.has_value())
                return LineRefusal{ sample.fault, Time(index) };
        }
        Restart(from);
        return std::nullopt;
    }

    void JointLinePlan::Build(const ToolPose& start, const ToolPose& target)
    {
        const auto length = Length(target.translation - start.translation);
        const auto angle = RotationAngle(start, target);
        std::optional<motion_profiles::SCurveProfile<float>> law;
        if (length > shortest)
            law = TimeLaw(length, tool.speedMPerS, tool.accelerationMPerS2, tool.jerkMPerS3);
        if (angle > shortest)
        {
            const auto turning = TimeLaw(angle, tool.angularSpeedRadPerS, tool.angularAccelerationRadPerS2, tool.angularJerkRadPerS3);
            if (!law.has_value() || turning.Duration() > law->Duration())
                law = turning;
        }

        if (!law.has_value())
            law = motion_profiles::SCurveProfile<float>{ 0.0f, 0.0f, { 1.0f, 1.0f, 1.0f } };
        line.emplace(start, target, *law);
        periods = std::max<std::size_t>(static_cast<std::size_t>(math::Ceil(line->Duration() / period)), 1);
    }

    void JointLinePlan::Restart(const JointPositions& from)
    {
        previous = from;
        previousSpeeds = {};
        index = 0;
    }

    LineSample JointLinePlan::Advance()
    {
        ++index;
        const auto pose = line->Sample(std::min(Time(index), line->Duration())).pose;
        const auto solution = model.Inverse(pose, previous);
        if (!solution.joints.has_value())
            return { std::nullopt, solution.failure == IkFailure::unreachable ? LineFault::unreachable : LineFault::outsideLimits };

        JointReference reference{};
        reference.positions = *solution.joints;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto speed = (reference.positions[joint].Value() - previous[joint].Value()) / period;
            reference.speeds[joint] = foc::RadiansPerSecond{ speed };
            reference.accelerations[joint] = foc::RadiansPerSecondSquared{ (speed - previousSpeeds[joint].Value()) / period };
        }

        if (const auto fault = Check(reference); fault.has_value())
            return { std::nullopt, *fault };

        previous = reference.positions;
        previousSpeeds = reference.speeds;
        return { reference, LineFault::unreachable };
    }

    std::optional<LineFault> JointLinePlan::Check(const JointReference& reference) const
    {
        if (model.Manipulability(reference.positions) < tool.manipulabilityMin)
            return LineFault::nearSingularity;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (math::Abs(reference.speeds[joint].Value()) > joints[joint].maxSpeed.Value() ||
                math::Abs(reference.accelerations[joint].Value()) > joints[joint].maxAcceleration.Value())
                return LineFault::tooFast;
        if (guard != nullptr && guard->Check(reference.positions).has_value())
            return LineFault::workspace;
        return std::nullopt;
    }

    bool JointLinePlan::Done() const
    {
        return index >= periods;
    }

    float JointLinePlan::Time(std::size_t sample) const
    {
        return static_cast<float>(sample) * period;
    }
}
