#include "tools/arm_host/ArmTargetLink.hpp"

namespace arm_host
{
    infra::Duration ArmTargetLink::ConnectTimeout() const
    {
        return std::chrono::seconds{ 2 };
    }

    std::optional<std::string> ArmTargetLink::Failure()
    {
        return std::nullopt;
    }

    VirtualMotors* ArmTargetLink::Motors()
    {
        return nullptr;
    }

    bool ArmTargetLink::Adopt(const arm::ArmConfig&)
    {
        return false;
    }

    simulation::PlantConfig PlantFor(const arm::ArmConfig& config)
    {
        return simulation::PlantFor(config.transmissions, config.geometry, config.mass, config.motors);
    }

    SimulatedTargetLink::SimulatedTargetLink(const arm::JointNodeIds& nodes, infra::Duration timeConstant)
        : arm(nodes, timeConstant)
    {}

    SimulatedTargetLink::SimulatedTargetLink(const arm::ArmConfig& config, infra::Duration timeConstant)
        : arm(config.nodes, timeConstant, PlantFor(config))
    {}

    hal::Can& SimulatedTargetLink::Can()
    {
        return arm.ArmSide();
    }

    bool SimulatedTargetLink::Connected()
    {
        return true;
    }

    void SimulatedTargetLink::Poll()
    {}

    simulation::SimulatedArm* SimulatedTargetLink::Simulation()
    {
        return &arm;
    }
}
