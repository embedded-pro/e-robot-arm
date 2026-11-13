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

    arm::ControllerSettings SimulatedJoint::Controller() const
    {
        arm::ControllerSettings settings{ node.algorithms.current, node.algorithms.speed, node.algorithms.position, {} };
        for (std::size_t gain = 0; gain != arm::gainCount; ++gain)
            settings.gains[gain] = node.parameters[static_cast<std::size_t>(arm::gainParameters[gain])];
        return settings;
    }

    SimulatedJoint::Node::Node(can::FocMotorCategoryServer& server, infra::Duration timeConstant)
        : can::FocMotorCategoryServerObserver(server)
        , server(server)
        , timeConstant(timeConstant)
    {
        const arm::ControllerSettings defaults{};
        for (std::size_t gain = 0; gain != arm::gainCount; ++gain)
            parameters[static_cast<std::size_t>(arm::gainParameters[gain])] = defaults.gains[gain];
    }

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

    void SimulatedJoint::Node::OnSelectAlgorithm(can::FocControlLoop loop, uint8_t algorithm, const infra::Function<void(const can::FocAlgorithms&)>& onDone)
    {
        if (loop == can::FocControlLoop::current)
            algorithms.current = static_cast<can::FocCurrentAlgorithm>(algorithm);
        else if (loop == can::FocControlLoop::speed)
            algorithms.speed = static_cast<can::FocSpeedAlgorithm>(algorithm);
        else
            algorithms.position = static_cast<can::FocPositionAlgorithm>(algorithm);
        OnQueryAlgorithms(onDone);
    }

    void SimulatedJoint::Node::OnQueryAlgorithms(const infra::Function<void(const can::FocAlgorithms&)>& onDone)
    {
        algorithms.mode = mode;
        onDone(algorithms);
    }

    void SimulatedJoint::Node::OnSetParameter(can::FocParameter parameter, float value, const infra::Function<void(can::FocParameterValue)>& onDone)
    {
        parameters[static_cast<std::size_t>(parameter)] = value;
        onDone(can::FocParameterValue{ value, false });
    }

    void SimulatedJoint::Node::OnQueryParameter(can::FocParameter parameter, const infra::Function<void(can::FocParameterValue)>& onDone)
    {
        onDone(can::FocParameterValue{ parameters[static_cast<std::size_t>(parameter)], false });
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
