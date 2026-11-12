#ifndef INTEGRATION_TESTS_SUPPORT_SIMULATED_JOINT_HPP
#define INTEGRATION_TESTS_SUPPORT_SIMULATED_JOINT_HPP

#include "can-lite/server/CanProtocolServer.hpp"
#include "e-foc-protocol/server/FocMotorCategoryServer.hpp"
#include "integration_tests/support/VirtualCanBus.hpp"
#include <cstdint>

namespace integration
{
    class SimulatedJoint
    {
    public:
        SimulatedJoint(VirtualCanBus& bus, uint16_t nodeId);
        SimulatedJoint(const SimulatedJoint& other) = delete;
        SimulatedJoint& operator=(const SimulatedJoint& other) = delete;
        ~SimulatedJoint();

        void Fault(can::FocFaultCode code);
        void Silence();

        can::FocMotorState State() const;
        can::FocMotorMode Mode() const;
        foc::Radians Position() const;
        uint32_t EmergencyStops() const;

    private:
        class Node
            : private can::FocMotorCategoryServerObserver
        {
        public:
            explicit Node(can::FocMotorCategoryServer& server);

            can::FocMotorState state{ can::FocMotorState::ready };
            can::FocFaultCode fault{ can::FocFaultCode::none };
            can::FocMotorMode mode{ can::FocMotorMode::torque };
            foc::Radians position{ 0.0f };
            uint32_t emergencyStops{ 0 };

        private:
            void OnStart(const infra::Function<void(services::CanAckStatus)>& onDone) override;
            void OnStop(const infra::Function<void(services::CanAckStatus)>& onDone) override;
            void OnClearFault(const infra::Function<void(services::CanAckStatus)>& onDone) override;
            void OnEmergencyStop(const infra::Function<void(services::CanAckStatus)>& onDone) override;
            void OnSelectControlMode(can::FocMotorMode newMode, const infra::Function<void(can::FocMotorMode)>& onActivated) override;
            void OnSetSetpoint(const can::FocSetpoint& setpoint, const infra::Function<void()>& onDone) override;
            void OnAlign(const infra::Function<void(services::CanAckStatus)>& onDone) override;
            void OnIdentifyElectrical(const infra::Function<void()>& onDone) override;
            void OnRequestTelemetry(const infra::Function<void()>& onDone) override;
            void OnSelectAlgorithm(can::FocControlLoop loop, uint8_t algorithm, const infra::Function<void(const can::FocAlgorithms&)>& onDone) override;
            void OnQueryAlgorithms(const infra::Function<void(const can::FocAlgorithms&)>& onDone) override;
            void OnSetParameter(can::FocParameter parameter, float value, const infra::Function<void(can::FocParameterValue)>& onDone) override;
            void OnQueryParameter(can::FocParameter parameter, const infra::Function<void(can::FocParameterValue)>& onDone) override;
            void OnCalibrate(const infra::Function<void(services::CanAckStatus)>& onDone) override;
            void OnClearCalibration(const infra::Function<void(services::CanAckStatus)>& onDone) override;
            void OnClearDiagnostics(const infra::Function<void()>& onDone) override;
            void OnQueryFaultHistory(uint8_t age, const infra::Function<void(const can::FocFaultRecord&)>& onDone) override;
            void OnClearFaultHistory(const infra::Function<void(bool stored)>& onDone) override;
            void OnRecalibrate(const infra::Function<void()>& onAccepted) override;
            void OnResetModel(const infra::Function<void(bool stored)>& onDone) override;

            can::FocMotorCategoryServer& server;
        };

        VirtualCanBus::Port port;
        services::CanProtocolServer::Config config;
        services::CanProtocolServer protocolServer;
        can::FocMotorCategoryServer server;
        Node node;
    };
}

#endif
