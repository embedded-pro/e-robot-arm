#include "core/motion/implementations/JointJogger.hpp"
#include "numerical/math/Math.hpp"
#include <algorithm>

namespace arm
{
    namespace
    {
        constexpr float atRest{ 1.0e-6f };
    }

    JointJogger::JointJogger(const JointLimits& positionLimits, const JointMotionLimits& limits, float period)
        : positionLimits(positionLimits)
        , limits(limits)
        , period(period)
    {}

    void JointJogger::Start(const JointPositions& from, const JointJog& jog)
    {
        positions = from;
        joint = jog.joint;
        speed = 0.0f;
        commanded = jog.speed.Value();
        silentPeriods = 0;
        pending.reset();
        done = false;
    }

    void JointJogger::Command(const JointJog& jog)
    {
        silentPeriods = 0;
        if (jog.joint == joint && !pending.has_value())
            commanded = jog.speed.Value();
        else
            pending = jog;
    }

    JointReference JointJogger::Advance()
    {
        ++silentPeriods;
        const auto acceleration = limits[joint].maxAcceleration.Value();
        const auto target = TargetSpeed();
        const auto change = std::clamp(target - speed, -acceleration * period, acceleration * period);
        const auto next = speed + change;
        const auto& limit = positionLimits[joint];
        positions[joint] = foc::Radians{ std::clamp(positions[joint].Value() + 0.5f * (speed + next) * period, limit.lower.Value(), limit.upper.Value()) };
        speed = next;

        JointReference reference{};
        reference.positions = positions;
        reference.speeds[joint] = foc::RadiansPerSecond{ speed };
        reference.accelerations[joint] = foc::RadiansPerSecondSquared{ change / period };

        if (math::Abs(speed) < atRest && math::Abs(target) < atRest)
        {
            if (pending.has_value() && silentPeriods <= jogDeadManPeriods)
            {
                joint = pending->joint;
                commanded = pending->speed.Value();
                pending.reset();
            }
            else
                done = true;
        }
        return reference;
    }

    bool JointJogger::Done() const
    {
        return done;
    }

    float JointJogger::TargetSpeed() const
    {
        if (pending.has_value() || silentPeriods > jogDeadManPeriods)
            return 0.0f;

        const auto maximum = limits[joint].maxSpeed.Value();
        const auto wanted = std::clamp(commanded, -maximum, maximum);
        const auto direction = wanted < 0.0f ? -1.0f : 1.0f;
        const auto capped = std::min(math::Abs(wanted), StoppingSpeed(direction));
        if (capped < 0.5f * limits[joint].maxAcceleration.Value() * period)
            return 0.0f;
        return direction * capped;
    }

    float JointJogger::StoppingSpeed(float direction) const
    {
        const auto& limit = positionLimits[joint];
        const auto position = positions[joint].Value();
        const auto room = direction > 0.0f ? limit.upper.Value() - position : position - limit.lower.Value();
        return math::Sqrt(2.0f * limits[joint].maxAcceleration.Value() * std::max(room, 0.0f));
    }
}
