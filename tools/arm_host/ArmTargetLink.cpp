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
        simulation::PlantConfig plant{ config.transmissions, config.geometry, config.mass };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto& motor = config.motors[joint].has_value() ? config.motors[joint]->settings : arm::motorPresets[0].settings;
            plant.rotorInertia[joint] = motor[arm::MotorParameter::inertia];
            plant.motorFriction[joint] = motor[arm::MotorParameter::viscousFriction];
        }
        return plant;
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
