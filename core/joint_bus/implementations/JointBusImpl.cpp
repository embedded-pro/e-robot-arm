#include "core/joint_bus/implementations/JointBusImpl.hpp"
#include "can-lite/core/CanProtocolDefinitions.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>

namespace arm
{
    namespace
    {
        bool ValidNodeMap(const JointNodeIds& nodes)
        {
            for (std::size_t joint = 0; joint != nodes.size(); ++joint)
            {
                const auto node = nodes[joint];
                if (node == services::canBroadcastNodeId || node > services::canIdNodeIdMask)
                    return false;
                if (std::ranges::find(nodes.begin() + joint + 1, nodes.end(), node) != nodes.end())
                    return false;
            }

            return true;
        }
    }

    JointBusImpl::JointBusImpl(can::FocMotorCategoryClient& client, const JointNodeIds& nodes, uint8_t missedRequestsBeforeSilent)
        : can::FocMotorCategoryClientObserver(client)
        , client(client)
        , nodes(nodes)
        , missedRequestsBeforeSilent(missedRequestsBeforeSilent)
    {
        really_assert(ValidNodeMap(nodes));
        really_assert(missedRequestsBeforeSilent != 0);
    }

    bool JointBusImpl::QueryContracts()
    {
        contractAnswered.fill(false);
        contractCheckPending = true;

        return ForEachJoint([this](std::size_t, uint16_t node)
            {
                return client.SendQueryContractVersion(node);
            });
    }

    can::ContractCompatibility JointBusImpl::Contracts() const
    {
        auto compatibility = can::ContractCompatibility::compatible;

        for (const auto node : nodes)
        {
            const auto nodeCompatibility = client.Compatibility(node);
            if (nodeCompatibility == can::ContractCompatibility::incompatible)
                return nodeCompatibility;
            if (nodeCompatibility == can::ContractCompatibility::unknown)
                compatibility = nodeCompatibility;
        }

        return compatibility;
    }

    bool JointBusImpl::SelectMode(can::FocMotorMode mode)
    {
        return ForEachJoint([this, mode](std::size_t, uint16_t node)
            {
                return client.SendSelectControlMode(node, mode);
            });
    }

    bool JointBusImpl::Start()
    {
        return ForEachJoint([this](std::size_t, uint16_t node)
            {
                return client.SendStart(node);
            });
    }

    bool JointBusImpl::Stop()
    {
        return ForEachJoint([this](std::size_t, uint16_t node)
            {
                return client.SendStop(node);
            });
    }

    bool JointBusImpl::ClearFault()
    {
        return ForEachJoint([this](std::size_t, uint16_t node)
            {
                return client.SendClearFault(node);
            });
    }

    bool JointBusImpl::EmergencyStop()
    {
        return ForEachJoint([this](std::size_t, uint16_t node)
            {
                return client.SendEmergencyStop(node);
            });
    }

    bool JointBusImpl::SendPositions(const JointPositions& positions)
    {
        return ForEachJoint([this, &positions](std::size_t joint, uint16_t node)
            {
                return client.SendSetpoint(node, can::FocSetpoint{ can::FocSetpointKind::position, positions[joint].Value() });
            });
    }

    bool JointBusImpl::RequestTelemetry()
    {
        if (telemetryRequested)
            CountMissedTelemetry();

        telemetryRequested = true;
        for (auto& joint : liveness)
            joint.answered = false;

        return ForEachJoint([this](std::size_t, uint16_t node)
            {
                return client.SendRequestTelemetry(node);
            });
    }

    const JointReading& JointBusImpl::Reading(std::size_t joint) const
    {
        really_assert(joint < jointCount);
        return readings[joint];
    }

    void JointBusImpl::OnSelectControlModeResponse(uint16_t, can::FocMotorMode)
    {}

    void JointBusImpl::OnCategoryError(uint16_t, uint8_t, can::FocMotorCategoryError)
    {}

    void JointBusImpl::OnTelemetryStatus(uint16_t sourceNodeId, can::FocMotorState state, can::FocFaultCode fault)
    {
        const auto joint = JointOf(sourceNodeId);
        if (!joint.has_value())
            return;

        auto& reading = readings[*joint];
        const bool enteredFault = state == can::FocMotorState::fault && reading.state != can::FocMotorState::fault;
        reading.state = state;
        reading.fault = fault;

        if (enteredFault)
            NotifyObservers([joint = *joint, fault](auto& observer)
                {
                    observer.OnJointFault(joint, fault);
                });
    }

    void JointBusImpl::OnTelemetryMotion(uint16_t sourceNodeId, foc::RadiansPerSecond speed, foc::Radians position)
    {
        const auto joint = JointOf(sourceNodeId);
        if (!joint.has_value())
            return;

        readings[*joint].speed = speed;
        readings[*joint].position = position;
        liveness[*joint] = Liveness{ true, 0, false };
    }

    void JointBusImpl::OnTelemetryCurrents(uint16_t, uint8_t, foc::Ampere, foc::Ampere, foc::Radians)
    {}

    void JointBusImpl::OnContractVersionResponse(uint16_t sourceNodeId, uint8_t, uint8_t)
    {
        const auto joint = JointOf(sourceNodeId);
        if (!joint.has_value() || !contractCheckPending)
            return;

        contractAnswered[*joint] = true;

        if (client.Compatibility(sourceNodeId) == can::ContractCompatibility::incompatible)
            ResolveContracts(can::ContractCompatibility::incompatible);
        else if (std::ranges::find(contractAnswered, false) == contractAnswered.end())
            ResolveContracts(can::ContractCompatibility::compatible);
    }

    void JointBusImpl::OnAlgorithmsResponse(uint16_t, const can::FocAlgorithms&)
    {}

    void JointBusImpl::OnParameterResponse(uint16_t, can::FocParameter, can::FocParameterValue)
    {}

    void JointBusImpl::OnFaultRecordResponse(uint16_t, uint8_t, can::FocFaultCode, can::FocMotorMode, uint32_t)
    {}

    void JointBusImpl::OnFaultConditionsResponse(uint16_t, uint8_t, int16_t, foc::Volts, foc::Ampere)
    {}

    bool JointBusImpl::ForEachJoint(const infra::Function<bool(std::size_t joint, uint16_t node)>& send)
    {
        bool allAccepted = true;

        for (std::size_t joint = 0; joint != jointCount; ++joint)
            allAccepted = send(joint, nodes[joint]) && allAccepted;

        return allAccepted;
    }

    std::optional<std::size_t> JointBusImpl::JointOf(uint16_t node) const
    {
        const auto found = std::ranges::find(nodes, node);
        if (found == nodes.end())
            return std::nullopt;

        return static_cast<std::size_t>(std::distance(nodes.begin(), found));
    }

    void JointBusImpl::CountMissedTelemetry()
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            auto& state = liveness[joint];
            if (state.answered || state.silent)
                continue;

            ++state.missed;
            if (state.missed == missedRequestsBeforeSilent)
            {
                state.silent = true;
                NotifyObservers([joint](auto& observer)
                    {
                        observer.OnJointSilent(joint);
                    });
            }
        }
    }

    void JointBusImpl::ResolveContracts(can::ContractCompatibility compatibility)
    {
        contractCheckPending = false;
        NotifyObservers([compatibility](auto& observer)
            {
                observer.OnContractsResolved(compatibility);
            });
    }
}
