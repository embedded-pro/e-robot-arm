#include "tools/simulated_arm/SimulatedJoint.hpp"
#include "tools/simulated_arm/FirstOrderLag.hpp"

namespace simulation
{
    SimulatedJoint::SimulatedJoint(VirtualCanBus& bus, uint16_t nodeId, infra::Duration timeConstant)
        : port(bus)
        , config{ nodeId }
        , protocolServer(port, config)
        , server(protocolServer.Transport())
        , node(server, timeConstant)
    {
        protocolServer.RegisterCategory(server);
    }

    SimulatedJoint::~SimulatedJoint()
    {
        protocolServer.UnregisterCategory(server);
    }

    void SimulatedJoint::Fault(can::FocFaultCode code)
    {
        node.state = can::FocMotorState::fault;
        node.fault = code;
        node.Hold();
    }

    void SimulatedJoint::Silence()
    {
        port.Mute();
    }

    can::FocMotorState SimulatedJoint::State() const
    {
        return node.state;
    }

    can::FocMotorMode SimulatedJoint::Mode() const
    {
        return node.mode;
    }

    foc::Radians SimulatedJoint::Position() const
    {
        return node.position;
    }

    foc::Radians SimulatedJoint::Target() const
    {
        return node.target;
    }

    uint32_t SimulatedJoint::EmergencyStops() const
    {
        return node.emergencyStops;
    }

    SimulatedJoint::Node::Node(can::FocMotorCategoryServer& server, infra::Duration timeConstant)
        : can::FocMotorCategoryServerObserver(server)
        , server(server)
        , timeConstant(timeConstant)
        , lastUpdate(infra::Now())
    {}

    void SimulatedJoint::Node::Advance()
    {
        const auto now = infra::Now();
        const auto followed = FollowWithLag(position, target, now - lastUpdate, timeConstant);
        position = followed.position;
        speed = followed.speed;
        lastUpdate = now;
    }

    void SimulatedJoint::Node::Hold()
    {
        Advance();
        target = position;
        speed = foc::RadiansPerSecond{ 0.0f };
    }

    void SimulatedJoint::Node::OnStart(const infra::Function<void(services::CanAckStatus)>& onDone)
    {
        if (state != can::FocMotorState::ready)
        {
            onDone(services::CanAckStatus::invalidState);
            return;
        }

        state = can::FocMotorState::running;
        onDone(services::CanAckStatus::success);
    }

    void SimulatedJoint::Node::OnStop(const infra::Function<void(services::CanAckStatus)>& onDone)
    {
        if (state == can::FocMotorState::running)
            state = can::FocMotorState::ready;
        Hold();
        onDone(services::CanAckStatus::success);
    }

    void SimulatedJoint::Node::OnClearFault(const infra::Function<void(services::CanAckStatus)>& onDone)
    {
        if (state == can::FocMotorState::fault)
        {
            state = can::FocMotorState::ready;
            fault = can::FocFaultCode::none;
        }
        onDone(services::CanAckStatus::success);
    }

    void SimulatedJoint::Node::OnEmergencyStop(const infra::Function<void(services::CanAckStatus)>& onDone)
    {
        ++emergencyStops;
        if (state == can::FocMotorState::running)
            state = can::FocMotorState::ready;
        Hold();
        onDone(services::CanAckStatus::success);
    }

    void SimulatedJoint::Node::OnSelectControlMode(can::FocMotorMode newMode, const infra::Function<void(can::FocMotorMode)>& onActivated)
    {
        mode = newMode;
        onActivated(mode);
    }

    void SimulatedJoint::Node::OnSetSetpoint(const can::FocSetpoint& setpoint, const infra::Function<void()>& onDone)
    {
        if (state == can::FocMotorState::running && mode == can::FocMotorMode::position && setpoint.kind == can::FocSetpointKind::position)
        {
            Advance();
            target = foc::Radians{ setpoint.value };
            Advance();
        }
        onDone();
    }

    void SimulatedJoint::Node::OnAlign(const infra::Function<void(services::CanAckStatus)>& onDone)
    {
        onDone(services::CanAckStatus::notImplemented);
    }

    void SimulatedJoint::Node::OnIdentifyElectrical(const infra::Function<void()>& onDone)
    {
        onDone();
    }

    void SimulatedJoint::Node::OnRequestTelemetry(const infra::Function<void()>& onDone)
    {
        Advance();
        server.BroadcastTelemetryStatus(state, fault);
        server.BroadcastTelemetryMotion(speed, position);
        onDone();
    }

    void SimulatedJoint::Node::OnSelectAlgorithm(can::FocControlLoop, uint8_t, const infra::Function<void(const can::FocAlgorithms&)>& onDone)
    {
        onDone(can::FocAlgorithms{});
    }

    void SimulatedJoint::Node::OnQueryAlgorithms(const infra::Function<void(const can::FocAlgorithms&)>& onDone)
    {
        onDone(can::FocAlgorithms{});
    }

    void SimulatedJoint::Node::OnSetParameter(can::FocParameter, float value, const infra::Function<void(can::FocParameterValue)>& onDone)
    {
        onDone(can::FocParameterValue{ value, false });
    }

    void SimulatedJoint::Node::OnQueryParameter(can::FocParameter, const infra::Function<void(can::FocParameterValue)>& onDone)
    {
        onDone(can::FocParameterValue{});
    }

    void SimulatedJoint::Node::OnCalibrate(const infra::Function<void(services::CanAckStatus)>& onDone)
    {
        onDone(services::CanAckStatus::notImplemented);
    }

    void SimulatedJoint::Node::OnClearCalibration(const infra::Function<void(services::CanAckStatus)>& onDone)
    {
        onDone(services::CanAckStatus::notImplemented);
    }

    void SimulatedJoint::Node::OnClearDiagnostics(const infra::Function<void()>& onDone)
    {
        onDone();
    }

    void SimulatedJoint::Node::OnQueryFaultHistory(uint8_t age, const infra::Function<void(const can::FocFaultRecord&)>& onDone)
    {
        can::FocFaultRecord record{};
        record.age = age;
        onDone(record);
    }

    void SimulatedJoint::Node::OnClearFaultHistory(const infra::Function<void(bool stored)>& onDone)
    {
        onDone(true);
    }

    void SimulatedJoint::Node::OnRecalibrate(const infra::Function<void()>& onAccepted)
    {
        onAccepted();
    }

    void SimulatedJoint::Node::OnResetModel(const infra::Function<void(bool stored)>& onDone)
    {
        onDone(true);
    }
}
