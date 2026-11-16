#ifndef TOOLS_ARM_HOST_ARM_TARGET_LINK_HPP
#define TOOLS_ARM_HOST_ARM_TARGET_LINK_HPP

#include "core/arm/ArmConfig.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "hal/interfaces/Can.hpp"
#include "tools/arm_host/ArmTarget.hpp"
#include "tools/arm_host/VirtualMotors.hpp"
#include "tools/simulated_arm/SimulatedArm.hpp"
#include <functional>
#include <memory>
#include <optional>
#include <string>

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
        virtual bool Connected() = 0;
        virtual void Poll() = 0;
        virtual simulation::SimulatedArm* Simulation() = 0;

        virtual infra::Duration ConnectTimeout() const;
        virtual std::optional<std::string> Failure();
        virtual VirtualMotors* Motors();
        virtual bool Adopt(const arm::ArmConfig& config);
    };

    using TargetOpener = std::function<std::unique_ptr<ArmTargetLink>(const ArmTarget& target, const arm::ArmConfig& config)>;

    class SimulatedTargetLink
        : public ArmTargetLink
    {
    public:
        SimulatedTargetLink(const arm::JointNodeIds& nodes, infra::Duration timeConstant);

        hal::Can& Can() override;
        bool Connected() override;
        void Poll() override;
        simulation::SimulatedArm* Simulation() override;

    private:
        simulation::SimulatedArm arm;
    };
}

#endif
