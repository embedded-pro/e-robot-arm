#include "tools/arm_host/ArmTargetLink.hpp"

namespace arm_host
{
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
