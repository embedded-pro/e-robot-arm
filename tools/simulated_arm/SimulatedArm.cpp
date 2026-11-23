#include "tools/simulated_arm/SimulatedArm.hpp"

namespace simulation
{
    SimulatedArm::SimulatedArm(const arm::JointNodeIds& nodes, infra::Duration timeConstant)
    {
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            joints[joint] = std::make_unique<SimulatedJoint>(bus, nodes[joint], timeConstant);
    }

    SimulatedArm::SimulatedArm(const arm::JointNodeIds& nodes, infra::Duration timeConstant, const PlantConfig& plantConfig)
        : SimulatedArm(nodes, timeConstant)
    {
        std::array<SimulatedJoint*, arm::jointCount> pointers{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            pointers[joint] = joints[joint].get();
        plant.emplace(plantConfig, pointers);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            joints[joint]->AttachPlant(*plant, joint);
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

    SimulatedHardwareStop& SimulatedArm::HardwareStop()
    {
        return hardwareStop;
    }

    void SimulatedArm::Push(std::size_t joint, foc::NewtonMeter torque)
    {
        if (plant)
            plant->Push(joint, torque);
    }
}
