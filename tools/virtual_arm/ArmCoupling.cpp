#include "tools/virtual_arm/ArmCoupling.hpp"
#include <algorithm>

namespace virtual_arm
{
    namespace
    {
        simulation::PlantConfig PlantOf(const arm::ArmConfig& config)
        {
            auto plant = simulation::PlantFor(config.transmissions, config.geometry, config.mass, config.motors);
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                const auto& motor = config.motors[joint].has_value() ? config.motors[joint]->settings : arm::motorPresets[0].settings;
                plant.coulombFriction[joint] = motor[arm::MotorParameter::coulombFriction];
            }
            return plant;
        }
    }

    ArmCoupling::ArmCoupling(const arm::ArmConfig& config)
        : transmissions(config.transmissions)
        , dynamics(PlantOf(config))
    {}

    ShaftSpeeds ArmCoupling::Step(const MotorSamples& motors, const arm::JointTorques& collision, float seconds)
    {
        const auto present = [](const std::optional<MotorSample>& motor)
        {
            return motor.has_value();
        };
        if (!engaged && !std::ranges::all_of(motors, present))
            return {};

        auto positions = dynamics.Positions();
        simulation::DrivingTorques torques{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (motors[joint].has_value())
            {
                positions[joint] = transmissions[joint].JointPosition(foc::Radians{ motors[joint]->angle });
                torques[joint] = foc::NewtonMeter{ motors[joint]->torque };
            }

        if (engaged)
            dynamics.SetPositions(positions);
        else
            dynamics.SetState(positions, arm::JointSpeeds{});
        engaged = true;

        dynamics.Step(torques, collision, seconds);
        const auto speeds = dynamics.Speeds();
        const auto inertias = dynamics.ApparentInertias(torques);
        ShaftSpeeds shafts{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (motors[joint].has_value())
            {
                const auto ratio = transmissions[joint].ratio;
                shafts[joint] = plant_port::ShaftDrive{ transmissions[joint].MotorSpeed(speeds[joint]).Value(), motors[joint]->period, motors[joint]->torque,
                    inertias[joint] / (ratio * ratio) };
            }
        return shafts;
    }

    bool ArmCoupling::Engaged() const
    {
        return engaged;
    }
}
