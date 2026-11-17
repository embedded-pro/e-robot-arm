#include "core/motion/implementations/JointMotionImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/Math.hpp"
#include <algorithm>
#include <chrono>

namespace arm
{
    namespace
    {
        bool ValidLimit(const JointMotionLimit& limit)
        {
            return limit.maxSpeed.Value() > 0.0f && limit.maxAcceleration.Value() > 0.0f;
        }

        bool Finite(const foc::Radians& position)
        {
            return math::IsFinite(position.Value());
        }
    }

    JointMotionImpl::JointMotionImpl(JointBus& bus, const JointLimits& positionLimits, const JointMotionLimits& limits, infra::Duration period)
        : bus(bus)
        , positionLimits(positionLimits)
        , limits(limits)
        , period(period)
    {
        really_assert(std::ranges::all_of(limits, ValidLimit));
        really_assert(period > infra::Duration::zero());
    }

    bool JointMotionImpl::MoveTo(const JointPositions& newTarget)
    {
        if (!Accepts(newTarget))
            return false;

        target = newTarget;
        plan.Plan(LatestPositions(), target, limits);
        periodsElapsed = 0;
        moving = true;
        stepping = false;
        return true;
    }

    bool JointMotionImpl::StepTo(const JointPositions& newTarget)
    {
        if (!Accepts(newTarget) || !WithinStep(newTarget))
            return false;

        target = newTarget;
        moving = true;
        stepping = true;
        return true;
    }

    bool JointMotionImpl::Accepts(const JointPositions& newTarget) const
    {
        return !Moving() && std::ranges::all_of(newTarget, Finite) && WithinLimits(positionLimits, newTarget) && !AnyReadingStale();
    }

    bool JointMotionImpl::WithinStep(const JointPositions& newTarget) const
    {
        const auto latest = LatestPositions();
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (math::Abs(newTarget[joint].Value() - latest[joint].Value()) > maxStep)
                return false;
        return true;
    }

    void JointMotionImpl::Abort()
    {
        if (!Moving())
            return;

        Finish();
        NotifyObservers([](auto& observer)
            {
                observer.OnMoveAborted();
            });
    }

    bool JointMotionImpl::Moving() const
    {
        return moving;
    }

    bool JointMotionImpl::AnyReadingStale() const
    {
        const auto period = bus.Period();
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (Stale(bus.Reading(joint), period))
                return true;

        return false;
    }

    JointPositions JointMotionImpl::LatestPositions() const
    {
        JointPositions positions{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            positions[joint] = bus.Reading(joint).position;
        return positions;
    }

    std::optional<JointReference> JointMotionImpl::Step()
    {
        if (!moving)
            return std::nullopt;

        ++periodsElapsed;
        const auto elapsed = std::chrono::duration<float>(period * periodsElapsed).count();
        const bool arrived = stepping || elapsed >= plan.Duration();
        const auto reference = arrived ? AtRest(target) : plan.Sample(elapsed);

        if (arrived)
        {
            Finish();
            NotifyObservers([](auto& observer)
                {
                    observer.OnMoveDone();
                });
        }

        return reference;
    }

    void JointMotionImpl::Finish()
    {
        moving = false;
    }
}
