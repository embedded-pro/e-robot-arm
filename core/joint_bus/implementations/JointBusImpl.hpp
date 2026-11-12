#ifndef CORE_JOINT_BUS_IMPLEMENTATIONS_JOINT_BUS_IMPL_HPP
#define CORE_JOINT_BUS_IMPLEMENTATIONS_JOINT_BUS_IMPL_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "e-foc-protocol/client/FocMotorCategoryClient.hpp"
#include "infra/timer/Timer.hpp"
#include "infra/util/Function.hpp"
#include <array>
#include <optional>

namespace arm
{
    class JointBusImpl
        : public JointBus
        , private can::FocMotorCategoryClientObserver
    {
    public:
        static constexpr infra::Duration retryInterval{ std::chrono::milliseconds{ 1 } };

        JointBusImpl(can::FocMotorCategoryClient& client, const JointNodeIds& nodes, uint8_t missedRequestsBeforeSilent, infra::Duration commandTimeout);

        bool QueryContracts() override;
        can::ContractCompatibility Contracts() const override;

        bool SelectMode(can::FocMotorMode mode) override;
        bool Start() override;
        bool Stop() override;
        bool ClearFault() override;
        bool EmergencyStop() override;
        bool SendPositions(const JointPositions& positions) override;
        bool RequestTelemetry() override;

        const JointReading& Reading(std::size_t joint) const override;

    private:
        void OnSelectControlModeResponse(uint16_t sourceNodeId, can::FocMotorMode activeMode) override;
        void OnCategoryError(uint16_t sourceNodeId, uint8_t originCommandId, can::FocMotorCategoryError errorCode) override;
        void OnTelemetryStatus(uint16_t sourceNodeId, can::FocMotorState state, can::FocFaultCode fault) override;
        void OnTelemetryMotion(uint16_t sourceNodeId, foc::RadiansPerSecond speed, foc::Radians position) override;
        void OnTelemetryCurrents(uint16_t sourceNodeId, uint8_t sequence, foc::Ampere id, foc::Ampere iq, foc::Radians position) override;
        void OnContractVersionResponse(uint16_t sourceNodeId, uint8_t major, uint8_t minor) override;
        void OnAlgorithmsResponse(uint16_t sourceNodeId, const can::FocAlgorithms& algorithms) override;
        void OnParameterResponse(uint16_t sourceNodeId, can::FocParameter parameter, can::FocParameterValue value) override;
        void OnFaultRecordResponse(uint16_t sourceNodeId, uint8_t age, can::FocFaultCode code, can::FocMotorMode mode, uint32_t uptimeMs) override;
        void OnFaultConditionsResponse(uint16_t sourceNodeId, uint8_t age, int16_t powerUp, foc::Volts busVoltage, foc::Ampere peakCurrent) override;

        struct HeldCommand
        {
            JointCommand command;
            can::FocMotorMode mode;
            std::array<bool, jointCount> accepted{};
        };

        bool Deliver(JointCommand command, can::FocMotorMode mode);
        void SendHeld();
        bool Send(const HeldCommand& command, uint16_t node);
        void Release();
        void Fail(CommandFailure failure);
        bool AnyJointIncompatible() const;
        bool ForEachJoint(const infra::Function<bool(std::size_t joint, uint16_t node)>& send);
        std::optional<std::size_t> JointOf(uint16_t node) const;
        void ResolveContracts(can::ContractCompatibility compatibility);
        void CountMissedTelemetry();

        can::FocMotorCategoryClient& client;
        JointNodeIds nodes;
        std::array<JointReading, jointCount> readings{};
        std::array<bool, jointCount> contractAnswered{};
        bool contractCheckPending{ false };

        struct Liveness
        {
            bool answered{ false };
            uint8_t missed{ 0 };
            bool silent{ false };
        };

        uint8_t missedRequestsBeforeSilent;
        std::array<Liveness, jointCount> liveness{};
        bool telemetryRequested{ false };

        infra::Duration commandTimeout;
        std::optional<HeldCommand> held;
        infra::TimerRepeating retryTimer;
        infra::TimerSingleShot timeoutTimer;
    };
}

#endif
