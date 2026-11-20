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

    JointMotionImpl::JointMotionImpl(JointBus& bus, const JointLimits& positionLimits, const JointMotionLimits& limits, infra::Duration period, const PoseGuard* guard,
        const JointJerkLimits& jerks)
        : bus(bus)
        , positionLimits(positionLimits)
        , limits(limits)
        , jerks(jerks)
        , period(period)
        , guard(guard)
    {
        really_assert(std::ranges::all_of(limits, ValidLimit));
        really_assert(period > infra::Duration::zero());
    }

    bool JointMotionImpl::MoveTo(const JointPositions& newTarget)
    {
        const bool replacing = moving && !stepping;
        if ((moving && !replacing) || !Acceptable(newTarget))
            return false;

        const auto from = replacing ? plan.Sample(Elapsed()) : AtRest(LatestPositions());
        if (!PathClear(from.positions, newTarget))
            return false;

        target = newTarget;
        if (replacing)
            plan.Replan(from, target, limits, jerks);
        else
            plan.Plan(from.positions, target, limits, jerks, profile);
        periodsElapsed = 0;
        moving = true;
        stepping = false;
        return true;
    }

    bool JointMotionImpl::StepTo(const JointPositions& newTarget)
    {
        if (!Accepts(newTarget) || !WithinStep(newTarget) || !PathClear(newTarget, newTarget))
            return false;

        target = newTarget;
        moving = true;
        stepping = true;
        return true;
    }

    bool JointMotionImpl::Accepts(const JointPositions& newTarget) const
    {
        return !Moving() && Acceptable(newTarget);
    }

    bool JointMotionImpl::Acceptable(const JointPositions& newTarget) const
    {
        return std::ranges::all_of(newTarget, Finite) && WithinLimits(positionLimits, newTarget) && !AnyReadingStale();
    }

    float JointMotionImpl::Elapsed() const
    {
        return std::chrono::duration<float>(period * periodsElapsed).count();
    }

    bool JointMotionImpl::PathClear(const JointPositions& from, const JointPositions& to) const
    {
        if (guard == nullptr)
            return true;

        float travel{ 0.0f };
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            travel = std::max(travel, math::Abs(to[joint].Value() - from[joint].Value()));
        const auto samples = std::clamp<std::size_t>(static_cast<std::size_t>(math::Ceil(travel / pathSampleSpacing)), 1, maxPathSamples);

        for (std::size_t sample = 1; sample <= samples; ++sample)
        {
            const auto fraction = static_cast<float>(sample) / static_cast<float>(samples);
            JointPositions pose{};
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                pose[joint] = foc::Radians{ from[joint].Value() + (to[joint].Value() - from[joint].Value()) * fraction };
            if (guard->Check(pose).has_value())
                return false;
        }
        return true;
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

    bool JointMotionImpl::SelectProfile(MotionProfile newProfile)
    {
        const auto jerkLimited = std::ranges::all_of(jerks, [](const std::optional<foc::RadiansPerSecondCubed>& jerk)
            {
                return jerk.has_value() && jerk->Value() > 0.0f;
            });
        if (Moving() || (newProfile == MotionProfile::sCurve && !jerkLimited))
            return false;

        profile = newProfile;
        return true;
    }

    MotionProfile JointMotionImpl::Profile() const
    {
        return profile;
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
        const auto elapsed = Elapsed();
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
