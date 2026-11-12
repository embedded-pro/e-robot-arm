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
        if (Moving() || !std::ranges::all_of(newTarget, Finite) || !WithinLimits(positionLimits, newTarget))
            return false;

        target = newTarget;
        plan.Plan(LatestPositions(), target, limits);
        periodsElapsed = 0;
        moving = true;
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

    JointPositions JointMotionImpl::LatestPositions() const
    {
        JointPositions positions{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            positions[joint] = bus.Reading(joint).position;
        return positions;
    }

    void JointMotionImpl::Step()
    {
        if (!moving)
            return;

        ++periodsElapsed;
        const auto elapsed = std::chrono::duration<float>(period * periodsElapsed).count();
        const bool arrived = elapsed >= plan.Duration();

        if (!bus.SendPositions(arrived ? target : plan.Sample(elapsed)))
            Abort();
        else if (arrived)
        {
            Finish();
            NotifyObservers([](auto& observer)
                {
                    observer.OnMoveDone();
                });
        }
    }

    void JointMotionImpl::Finish()
    {
        moving = false;
    }
}
