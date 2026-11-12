#include "tools/simulated_arm/SimulatedArm.hpp"

namespace simulation
{
    SimulatedArm::SimulatedArm(const arm::JointNodeIds& nodes, infra::Duration timeConstant)
    {
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            joints[joint] = std::make_unique<SimulatedJoint>(bus, nodes[joint], timeConstant);
    }

    hal::Can& SimulatedArm::ArmSide()
    {
        return armSide;
    }

    SimulatedJoint& SimulatedArm::Joint(std::size_t index)
    {
        return *joints.at(index);
    }

    bool SimulatedArm::Idle() const
    {
        return bus.Idle();
    }
}
