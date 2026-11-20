#include "core/motion/implementations/ToolJogger.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Quaternion.hpp"
#include <algorithm>

namespace arm
{
    namespace
    {
        constexpr float atRest{ 1.0e-6f };
        constexpr float jointMargin{ 0.98f };
        constexpr std::size_t extraBrakingSteps{ 50 };

        math::Vector3<float> Unit(ToolAxis axis)
        {
            math::Vector3<float> unit{};
            unit[static_cast<std::size_t>(axis) % 3] = 1.0f;
            return unit;
        }

        bool SameJog(const ToolJog& a, const ToolJog& b)
        {
            return a.frame == b.frame && a.axis == b.axis;
        }
    }

    ToolJogger::ToolJogger(const ArmModel& model, const ToolLimits& tool, const JointMotionLimits& joints, const PoseGuard* guard, float period)
        : model(model)
        , tool(tool)
        , joints(joints)
        , guard(guard)
        , period(period)
    {}

    void ToolJogger::Start(const JointPositions& from, const ToolJog& newJog)
    {
        jog = newJog;
        state = State{ model.ToolPose(from), from, {}, {}, 0.0f };
        silentPeriods = 0;
        pending.reset();
        done = false;
    }

    void ToolJogger::Command(const ToolJog& newJog)
    {
        silentPeriods = 0;
        if (SameJog(newJog, jog) && !pending.has_value())
            jog.speed = newJog.speed;
        else
            pending = newJog;
    }

    void ToolJogger::Stop()
    {
        jog.speed = 0.0f;
        pending.reset();
    }

    JointReference ToolJogger::Advance()
    {
        ++silentPeriods;
        const auto target = TargetSpeed();
        auto next = StepFrom(state, target);
        if (math::Abs(target) > atRest && (!next.has_value() || !Stoppable(*next)))
            next = StepFrom(state, 0.0f);
        if (!next.has_value())
            return Hold();

        state = *next;
        if (math::Abs(state.speed) < atRest && math::Abs(TargetSpeed()) < atRest)
        {
            if (pending.has_value() && silentPeriods <= jogDeadManPeriods)
            {
                jog = *pending;
                pending.reset();
                state.pose = model.ToolPose(state.joints);
            }
            else
                done = true;
        }
        return JointReference{ state.joints, state.speeds, state.accelerations };
    }

    std::optional<ToolJogger::State> ToolJogger::StepFrom(const State& from, float target) const
    {
        const auto speed = FeasibleSpeed(from, target);
        if (!speed.has_value())
            return std::nullopt;

        State next{ Moved(from.pose, *speed * period), {}, {}, {}, *speed };
        const auto solution = model.Inverse(next.pose, from.joints).joints;
        if (!solution.has_value() || !Acceptable(*solution))
            return std::nullopt;

        next.joints = *solution;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto jointSpeed = (next.joints[joint].Value() - from.joints[joint].Value()) / period;
            const auto jointAcceleration = (jointSpeed - from.speeds[joint].Value()) / period;
            if (math::Abs(jointSpeed) > joints[joint].maxSpeed.Value() || math::Abs(jointAcceleration) > joints[joint].maxAcceleration.Value())
                return std::nullopt;
            next.speeds[joint] = foc::RadiansPerSecond{ jointSpeed };
            next.accelerations[joint] = foc::RadiansPerSecondSquared{ jointAcceleration };
        }
        return next;
    }

    std::optional<float> ToolJogger::FeasibleSpeed(const State& from, float target) const
    {
        const auto step = AccelerationLimit() * period;
        SpeedRange range{ from.speed > 0.0f ? std::min(from.speed - step, 0.0f) : from.speed - step,
            from.speed < 0.0f ? std::max(from.speed + step, 0.0f) : from.speed + step };
        const auto probe = std::max(math::Abs(from.speed), step) * period;
        const auto probed = model.Inverse(Moved(from.pose, probe), from.joints).joints;
        if (!probed.has_value())
            return std::nullopt;

        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto gain = ((*probed)[joint].Value() - from.joints[joint].Value()) / probe;
            const auto maximum = jointMargin * joints[joint].maxSpeed.Value();
            const auto window = jointMargin * joints[joint].maxAcceleration.Value() * period;
            const auto current = from.speeds[joint].Value();
            range.Narrow(gain, -maximum, maximum);
            range.Narrow(gain, current - window, current + window);
        }

        if (range.low > range.high)
            return std::nullopt;
        return std::clamp(std::clamp(target, from.speed - step, from.speed + step), range.low, range.high);
    }

    void ToolJogger::SpeedRange::Narrow(float gain, float lower, float upper)
    {
        if (math::Abs(gain) < atRest)
        {
            if (lower > 0.0f || upper < 0.0f)
                low = high + 1.0f;
            return;
        }

        const auto first = lower / gain;
        const auto second = upper / gain;
        low = std::max(low, std::min(first, second));
        high = std::min(high, std::max(first, second));
    }

    bool ToolJogger::Stoppable(State from) const
    {
        const auto steps = static_cast<std::size_t>(math::Abs(from.speed) / (AccelerationLimit() * period)) + extraBrakingSteps;
        for (std::size_t step = 0; step != steps; ++step)
        {
            if (math::Abs(from.speed) < atRest)
                return true;
            const auto next = StepFrom(from, 0.0f);
            if (!next.has_value())
                return false;
            from = *next;
        }
        return math::Abs(from.speed) < atRest;
    }

    JointReference ToolJogger::Hold()
    {
        JointReference reference{};
        reference.positions = state.joints;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            reference.accelerations[joint] = foc::RadiansPerSecondSquared{ -state.speeds[joint].Value() / period };
        state.speeds = {};
        state.speed = 0.0f;
        done = true;
        return reference;
    }

    bool ToolJogger::Done() const
    {
        return done;
    }

    bool ToolJogger::Linear() const
    {
        return static_cast<std::size_t>(jog.axis) < 3;
    }

    float ToolJogger::SpeedLimit() const
    {
        return Linear() ? tool.speedMPerS : tool.angularSpeedRadPerS;
    }

    float ToolJogger::AccelerationLimit() const
    {
        return Linear() ? tool.accelerationMPerS2 : tool.angularAccelerationRadPerS2;
    }

    float ToolJogger::TargetSpeed() const
    {
        if (pending.has_value() || silentPeriods > jogDeadManPeriods)
            return 0.0f;
        return std::clamp(jog.speed, -SpeedLimit(), SpeedLimit());
    }

    ToolPose ToolJogger::Moved(const ToolPose& from, float distance) const
    {
        auto moved = from;
        const auto unit = Unit(jog.axis);
        if (Linear())
            moved.translation = from.translation + (jog.frame == JogFrame::tool ? from.rotation * unit : unit) * distance;
        else
        {
            const auto turn = math::Quaternion<float>::FromAxisAngle(unit, distance).ToRotationMatrix();
            moved.rotation = jog.frame == JogFrame::tool ? from.rotation * turn : turn * from.rotation;
        }
        return moved;
    }

    bool ToolJogger::Acceptable(const JointPositions& solution) const
    {
        return model.Manipulability(solution) >= tool.manipulabilityMin && (guard == nullptr || !guard->Check(solution).has_value());
    }
}
