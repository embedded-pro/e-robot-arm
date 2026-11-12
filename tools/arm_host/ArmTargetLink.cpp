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

    void SimulatedTargetLink::Poll()
    {}

    simulation::SimulatedArm* SimulatedTargetLink::Simulation()
    {
        return &arm;
    }
}
