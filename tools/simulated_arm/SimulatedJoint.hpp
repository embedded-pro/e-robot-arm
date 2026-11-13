#ifndef TOOLS_SIMULATED_ARM_SIMULATED_JOINT_HPP
#define TOOLS_SIMULATED_ARM_SIMULATED_JOINT_HPP

#include "can-lite/server/CanProtocolServer.hpp"
#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "e-foc-protocol/server/FocMotorCategoryServer.hpp"
#include "infra/timer/Timer.hpp"
#include "tools/simulated_arm/VirtualCanBus.hpp"
#include <array>
#include <cstdint>

namespace simulation
{
    class SimulatedJoint
    {
    public:
        SimulatedJoint(VirtualCanBus& bus, uint16_t nodeId, infra::Duration timeConstant = {});
        SimulatedJoint(const SimulatedJoint& other) = delete;
        SimulatedJoint& operator=(const SimulatedJoint& other) = delete;
        ~SimulatedJoint();

        void Fault(can::FocFaultCode code);
        void Silence();

        can::FocMotorState State() const;
        can::FocMotorMode Mode() const;
        foc::Radians Position() const;
        foc::Radians Target() const;
        uint32_t EmergencyStops() const;
        arm::ControllerSettings Controller() const;

    private:
        class Node
            : private can::FocMotorCategoryServerObserver
        {
        public:
            Node(can::FocMotorCategoryServer& server, infra::Duration timeConstant);

            void Advance();
            void Hold();

        private:
            friend class SimulatedJoint;

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

            can::FocMotorState state{ can::FocMotorState::ready };
            can::FocFaultCode fault{ can::FocFaultCode::none };
            can::FocMotorMode mode{ can::FocMotorMode::torque };
            foc::Radians position{ 0.0f };
            foc::RadiansPerSecond speed{ 0.0f };
            foc::Radians target{ 0.0f };
            uint32_t emergencyStops{ 0 };
            can::FocAlgorithms algorithms{};
            std::array<float, 256> parameters{};
            can::FocMotorCategoryServer& server;
            infra::Duration timeConstant;
            infra::TimePoint lastUpdate{ infra::Now() };
        };

        VirtualCanBus::Port port;
        services::CanProtocolServer::Config config;
        services::CanProtocolServer protocolServer;
        can::FocMotorCategoryServer server;
        Node node;
    };
}

#endif
