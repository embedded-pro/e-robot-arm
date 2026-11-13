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
    }

    SafetySupervisorImpl::SafetySupervisorImpl(JointBus& bus, const JointLimits& limits)
        : JointBusObserver(bus)
        , bus(bus)
        , limits(limits)
    {
        really_assert(std::ranges::all_of(limits, ValidLimit));
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

    bool SafetySupervisorImpl::EmergencyStop()
    {
        return bus.EmergencyStop();
    }

    bool SafetySupervisorImpl::SendPositions(const JointPositions& positions)
    {
        return MayCommandMotion() && WithinLimits(limits, positions) && bus.SendPositions(positions);
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

    void SafetySupervisorImpl::Trip()
    {
        if (tripped)
            return;

        tripped = true;
        bus.EmergencyStop();
    }
}
