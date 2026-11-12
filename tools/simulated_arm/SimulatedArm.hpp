#ifndef TOOLS_SIMULATED_ARM_SIMULATED_ARM_HPP
#define TOOLS_SIMULATED_ARM_SIMULATED_ARM_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "tools/simulated_arm/SimulatedJoint.hpp"
#include "tools/simulated_arm/VirtualCanBus.hpp"
#include <array>
#include <memory>

namespace simulation
{
    class SimulatedArm
    {
    public:
        SimulatedArm(const arm::JointNodeIds& nodes, infra::Duration timeConstant);
        SimulatedArm(const SimulatedArm& other) = delete;
        SimulatedArm& operator=(const SimulatedArm& other) = delete;

        hal::Can& ArmSide();
        SimulatedJoint& Joint(std::size_t index);
        bool Idle() const;

    private:
        VirtualCanBus bus;
        std::array<std::unique_ptr<SimulatedJoint>, arm::jointCount> joints;
        VirtualCanBus::Port armSide{ bus };
    };
}

#endif
