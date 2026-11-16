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

    SimulatedTargetLink::SimulatedTargetLink(const arm::JointNodeIds& nodes, infra::Duration timeConstant)
        : arm(nodes, timeConstant)
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
