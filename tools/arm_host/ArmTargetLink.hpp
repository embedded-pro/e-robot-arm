#ifndef TOOLS_ARM_HOST_ARM_TARGET_LINK_HPP
#define TOOLS_ARM_HOST_ARM_TARGET_LINK_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "hal/interfaces/Can.hpp"
#include "tools/arm_host/ArmTarget.hpp"
#include "tools/simulated_arm/SimulatedArm.hpp"
#include <functional>
#include <memory>

namespace arm_host
{
    class ArmTargetLink
    {
    public:
        ArmTargetLink() = default;
        ArmTargetLink(const ArmTargetLink& other) = delete;
        ArmTargetLink& operator=(const ArmTargetLink& other) = delete;
        virtual ~ArmTargetLink() = default;

        virtual hal::Can& Can() = 0;
        virtual void Poll() = 0;
        virtual simulation::SimulatedArm* Simulation() = 0;
    };

    using TargetOpener = std::function<std::unique_ptr<ArmTargetLink>(const ArmTarget& target)>;

    class SimulatedTargetLink
        : public ArmTargetLink
    {
    public:
        SimulatedTargetLink(const arm::JointNodeIds& nodes, infra::Duration timeConstant);

        hal::Can& Can() override;
        void Poll() override;
        simulation::SimulatedArm* Simulation() override;

    private:
        simulation::SimulatedArm arm;
    };
}

#endif
