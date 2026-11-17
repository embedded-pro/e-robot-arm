#include "core/safety/implementations/SafetySupervisorImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/Math.hpp"
#include <algorithm>

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
    }

    SafetySupervisorImpl::SafetySupervisorImpl(JointBus& bus, const JointLimits& limits, const TrackingLimits& tracking, infra::Duration period)
        : JointBusObserver(bus)
        , bus(bus)
        , limits(limits)
        , tracking(tracking)
    {
        really_assert(std::ranges::all_of(limits, ValidLimit));
        really_assert(std::ranges::all_of(tracking, ValidTracking));
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
        return MayCommandMotion() && finite && bus.SendTorques(torques);
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

    bool SafetySupervisorImpl::Rearm()
    {
        if (AnyJointFaulted())
            return false;

        tripped = false;
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
        return !tripped && bus.Contracts() == can::ContractCompatibility::compatible;
    }

    bool SafetySupervisorImpl::AnyJointFaulted() const
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (bus.Reading(joint).state == can::FocMotorState::fault)
                return true;

        return false;
    }

    void SafetySupervisorImpl::Supervise(const std::optional<JointPositions>& reference)
    {
        if (!reference.has_value() || tripped)
        {
            stepsOutside.fill(0);
            return;
        }

        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (!SuperviseJoint(joint, (*reference)[joint]))
                return;
    }

    bool SafetySupervisorImpl::SuperviseJoint(std::size_t joint, foc::Radians reference)
    {
        const auto& reading = bus.Reading(joint);
        if (Stale(reading, bus.Period()))
            return true;

        const auto measured = reading.position;
        if (measured < limits[joint].lower || limits[joint].upper < measured)
        {
            TripOnSupervision(joint, SupervisionTrip::positionLimit, measured);
            return false;
        }

        const foc::Radians error{ math::Abs(reference.Value() - measured.Value()) };
        stepsOutside[joint] = tracking[joint].window < error ? stepsOutside[joint] + 1 : 0;
        if (stepsOutside[joint] < stepsToTrip[joint])
            return true;

        TripOnSupervision(joint, SupervisionTrip::trackingError, error);
        return false;
    }

    void SafetySupervisorImpl::TripOnSupervision(std::size_t joint, SupervisionTrip cause, foc::Radians value)
    {
        Trip();
        stepsOutside.fill(0);
        NotifyObservers([joint, cause, value](auto& observer)
            {
                observer.OnSupervisionTrip(joint, cause, value);
            });
    }

    void SafetySupervisorImpl::Trip()
    {
        if (tripped)
            return;

        tripped = true;
        bus.EmergencyStop();
    }
}
