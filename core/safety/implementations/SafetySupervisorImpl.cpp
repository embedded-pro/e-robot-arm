#include "core/safety/implementations/SafetySupervisorImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/Math.hpp"
#include <algorithm>
#include <utility>

namespace arm
{
    namespace
    {
        bool ValidLimit(const JointLimit& limit)
        {
            return math::IsFinite(limit.lower.Value()) && math::IsFinite(limit.upper.Value()) && limit.lower < limit.upper;
        }

        bool ValidTracking(const TrackingLimit& limit)
        {
            return math::IsFinite(limit.window.Value()) && limit.window.Value() > 0.0f && limit.time > infra::Duration::zero();
        }

        constexpr float limitTolerance{ 1.0e-3f };

        SupervisionTrip CauseOf(WorkspaceRule rule)
        {
            switch (rule)
            {
                case WorkspaceRule::workspace:
                    return SupervisionTrip::workspace;
                case WorkspaceRule::forbiddenZone:
                    return SupervisionTrip::forbiddenZone;
                default:
                    return SupervisionTrip::selfCollision;
            }
        }

        bool Above(float value, float limit)
        {
            return math::Abs(value) > limit * (1.0f + limitTolerance) + 1.0e-6f;
        }
    }

    SafetySupervisorImpl::SafetySupervisorImpl(JointBus& bus, const JointLimits& limits, const TrackingLimits& tracking, const CollisionLimits& collision,
        infra::Duration period, const MotionSupervision& motion, const PoseGuard* guard)
        : JointBusObserver(bus)
        , bus(bus)
        , limits(limits)
        , tracking(tracking)
        , collision(collision)
        , motion(motion)
        , periodSeconds(std::chrono::duration<float>{ period }.count())
        , guard(guard)
    {
        really_assert(std::ranges::all_of(limits, ValidLimit));
        really_assert(std::ranges::all_of(tracking, ValidTracking));
        really_assert(std::ranges::all_of(collision, [](foc::NewtonMeter limit)
            {
                return limit.Value() > 0.0f;
            }));
        really_assert(period > infra::Duration::zero());

        for (std::size_t joint = 0; joint != jointCount; ++joint)
            stepsToTrip[joint] = static_cast<uint32_t>((tracking[joint].time + period - infra::Duration{ 1 }) / period);
    }

    bool SafetySupervisorImpl::QueryContracts()
    {
        return bus.QueryContracts();
    }

    can::ContractCompatibility SafetySupervisorImpl::Contracts() const
    {
        return bus.Contracts();
    }

    bool SafetySupervisorImpl::SelectMode(can::FocMotorMode mode)
    {
        return MayCommandMotion() && bus.SelectMode(mode);
    }

    bool SafetySupervisorImpl::Start()
    {
        return MayCommandMotion() && bus.Start();
    }

    bool SafetySupervisorImpl::Stop()
    {
        return bus.Stop();
    }

    bool SafetySupervisorImpl::ClearFault()
    {
        return bus.ClearFault();
    }

    bool SafetySupervisorImpl::Align()
    {
        return MayCommandMotion() && bus.Align();
    }

    bool SafetySupervisorImpl::EmergencyStop()
    {
        return bus.EmergencyStop();
    }

    bool SafetySupervisorImpl::SendPositions(const JointPositions& positions)
    {
        return MayCommandMotion() && WithinLimits(limits, positions) && bus.SendPositions(positions);
    }

    bool SafetySupervisorImpl::SendTorques(const JointTorques& torques)
    {
        const auto finite = std::ranges::all_of(torques, [](foc::NewtonMeter torque)
            {
                return math::IsFinite(torque.Value());
            });
        if (!MayStream() || !finite)
            return false;

        auto clamped = torques;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto limit = motion.torque[joint].Value();
            clamped[joint] = foc::NewtonMeter{ std::clamp(torques[joint].Value(), -limit, limit) };
            torqueClamped[joint] = clamped[joint] != torques[joint];
        }

        bus.SendTorques(clamped);
        return true;
    }

    const JointFlags& SafetySupervisorImpl::TorqueClamped() const
    {
        return torqueClamped;
    }

    std::optional<WorkspaceViolation> SafetySupervisorImpl::LastWorkspaceViolation() const
    {
        return lastWorkspaceViolation;
    }

    bool SafetySupervisorImpl::RequestTelemetry()
    {
        return bus.RequestTelemetry();
    }

    bool SafetySupervisorImpl::SelectAlgorithm(std::size_t joint, can::FocControlLoop loop, uint8_t algorithm)
    {
        return bus.SelectAlgorithm(joint, loop, algorithm);
    }

    bool SafetySupervisorImpl::QueryAlgorithms(std::size_t joint)
    {
        return bus.QueryAlgorithms(joint);
    }

    bool SafetySupervisorImpl::SetParameter(std::size_t joint, can::FocParameter parameter, float value)
    {
        return bus.SetParameter(joint, parameter, value);
    }

    bool SafetySupervisorImpl::QueryParameter(std::size_t joint, can::FocParameter parameter)
    {
        return bus.QueryParameter(joint, parameter);
    }

    const JointReading& SafetySupervisorImpl::Reading(std::size_t joint) const
    {
        return bus.Reading(joint);
    }

    uint32_t SafetySupervisorImpl::Period() const
    {
        return bus.Period();
    }

    bool SafetySupervisorImpl::Tripped() const
    {
        return tripped;
    }

    bool SafetySupervisorImpl::Holding() const
    {
        return holding;
    }

    bool SafetySupervisorImpl::Rearm()
    {
        if (AnyJointFaulted())
            return false;

        tripped = false;
        holding = false;
        stepsOutside.fill(0);
        previousReference.reset();
        return true;
    }

    void SafetySupervisorImpl::OnContractsResolved(can::ContractCompatibility compatibility)
    {
        NotifyObservers([compatibility](auto& observer)
            {
                observer.OnContractsResolved(compatibility);
            });
    }

    void SafetySupervisorImpl::OnJointFault(std::size_t joint, can::FocFaultCode fault)
    {
        Trip();
        NotifyObservers([joint, fault](auto& observer)
            {
                observer.OnJointFault(joint, fault);
            });
    }

    void SafetySupervisorImpl::OnJointSilent(std::size_t joint)
    {
        Trip();
        NotifyObservers([joint](auto& observer)
            {
                observer.OnJointSilent(joint);
            });
    }

    void SafetySupervisorImpl::OnCommandDelivered(JointCommand command)
    {
        NotifyObservers([command](auto& observer)
            {
                observer.OnCommandDelivered(command);
            });
    }

    void SafetySupervisorImpl::OnCommandFailed(JointCommand command, CommandFailure failure)
    {
        NotifyObservers([command, failure](auto& observer)
            {
                observer.OnCommandFailed(command, failure);
            });
    }

    void SafetySupervisorImpl::OnAlgorithms(std::size_t joint, const can::FocAlgorithms& algorithms)
    {
        NotifyObservers([joint, &algorithms](auto& observer)
            {
                observer.OnAlgorithms(joint, algorithms);
            });
    }

    void SafetySupervisorImpl::OnParameter(std::size_t joint, can::FocParameter parameter, can::FocParameterValue value)
    {
        NotifyObservers([joint, parameter, value](auto& observer)
            {
                observer.OnParameter(joint, parameter, value);
            });
    }

    bool SafetySupervisorImpl::MayCommandMotion() const
    {
        return !holding && MayStream();
    }

    bool SafetySupervisorImpl::MayStream() const
    {
        return !tripped && bus.Contracts() == can::ContractCompatibility::compatible;
    }

    bool SafetySupervisorImpl::AnyJointFaulted() const
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (bus.Reading(joint).state == can::FocMotorState::fault)
                return true;

        return false;
    }

    void SafetySupervisorImpl::Supervise(const std::optional<JointReference>& reference, const std::optional<JointTorques>& externalTorques)
    {
        if (!reference.has_value() || tripped)
        {
            stepsOutside.fill(0);
            previousReference.reset();
            return;
        }

        if (holding)
        {
            SuperviseHold(reference->positions);
            return;
        }

        if (externalTorques.has_value() && !SuperviseCollision(*externalTorques))
            return;

        if (!SuperviseReference(*reference) || !SuperviseWorkspace(reference->positions))
            return;

        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (!SuperviseJoint(joint, reference->positions[joint]))
                return;
    }

    bool SafetySupervisorImpl::SuperviseReference(const JointReference& reference)
    {
        const auto previous = std::exchange(previousReference, reference.positions);
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto& limit = motion.reference[joint];
            if (Above(reference.speeds[joint].Value(), limit.speed.Value()))
            {
                Protect(joint, SupervisionTrip::referenceSpeed, reference.speeds[joint].Value());
                return false;
            }
            if (Above(reference.accelerations[joint].Value(), limit.acceleration.Value()))
            {
                Protect(joint, SupervisionTrip::referenceAcceleration, reference.accelerations[joint].Value());
                return false;
            }
            const auto travelled = previous.has_value() ? reference.positions[joint].Value() - (*previous)[joint].Value() : 0.0f;
            if (Above(travelled, std::max(limit.speed.Value() * periodSeconds, maxReferenceStep)))
            {
                Protect(joint, SupervisionTrip::referenceStep, math::Abs(travelled));
                return false;
            }
        }
        return true;
    }

    bool SafetySupervisorImpl::SuperviseWorkspace(const JointPositions& reference)
    {
        if (guard == nullptr)
            return true;

        auto violation = guard->Check(reference);
        if (const auto measured = FreshMeasuredPose(); !violation.has_value() && measured.has_value())
            violation = guard->Check(*measured);
        if (!violation.has_value())
            return true;

        lastWorkspaceViolation = violation;
        Protect(violation->link, CauseOf(violation->rule), violation->distance);
        return false;
    }

    std::optional<JointPositions> SafetySupervisorImpl::FreshMeasuredPose() const
    {
        JointPositions measured{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto& reading = bus.Reading(joint);
            if (Stale(reading, bus.Period()))
                return std::nullopt;
            measured[joint] = reading.position;
        }
        return measured;
    }

    bool SafetySupervisorImpl::SuperviseSpeed(std::size_t joint)
    {
        const auto speed = bus.Reading(joint).speed.Value();
        if (!Above(speed, motion.speed[joint].Value()))
            return true;
        Protect(joint, SupervisionTrip::speedLimit, speed);
        return false;
    }

    bool SafetySupervisorImpl::SuperviseJoint(std::size_t joint, foc::Radians reference)
    {
        const auto& reading = bus.Reading(joint);
        if (Stale(reading, bus.Period()))
            return true;

        const auto measured = reading.position;
        if (measured < limits[joint].lower || limits[joint].upper < measured)
        {
            Protect(joint, SupervisionTrip::positionLimit, measured.Value());
            return false;
        }

        if (!SuperviseSpeed(joint))
            return false;

        const auto error = TrackingErrorHeld(joint, reference);
        if (!error.has_value())
            return true;

        Protect(joint, SupervisionTrip::trackingError, *error);
        return false;
    }

    std::optional<float> SafetySupervisorImpl::TrackingErrorHeld(std::size_t joint, foc::Radians reference)
    {
        const foc::Radians error{ math::Abs(reference.Value() - bus.Reading(joint).position.Value()) };
        stepsOutside[joint] = tracking[joint].window < error ? stepsOutside[joint] + 1 : 0;
        if (stepsOutside[joint] < stepsToTrip[joint])
            return std::nullopt;
        return error.Value();
    }

    void SafetySupervisorImpl::SuperviseHold(const JointPositions& held)
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            if (Stale(bus.Reading(joint), bus.Period()))
                continue;

            if (const auto error = TrackingErrorHeld(joint, held[joint]); error.has_value())
            {
                const auto value = *error;
                Trip();
                stepsOutside.fill(0);
                NotifyObservers([joint, value](auto& observer)
                    {
                        observer.OnSupervisionTrip(joint, SupervisionTrip::holdLost, value);
                    });
                return;
            }
        }
    }

    bool SafetySupervisorImpl::SuperviseCollision(const JointTorques& externalTorques)
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (collision[joint].Value() < math::Abs(externalTorques[joint].Value()))
            {
                Protect(joint, SupervisionTrip::collision, externalTorques[joint].Value());
                return false;
            }

        return true;
    }

    void SafetySupervisorImpl::Protect(std::size_t joint, SupervisionTrip cause, float value)
    {
        holding = true;
        stepsOutside.fill(0);
        previousReference.reset();
        NotifyObservers([joint, cause, value](auto& observer)
            {
                observer.OnProtectiveStop(joint, cause, value);
            });
    }

    void SafetySupervisorImpl::Trip()
    {
        holding = false;
        if (tripped)
            return;

        tripped = true;
        bus.EmergencyStop();
    }
}
