#include "tools/virtual_arm/ArmCoupling.hpp"
#include <algorithm>

namespace virtual_arm
{
    namespace
    {
        constexpr float longestStep{ 0.02f };

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

    ShaftSpeeds ArmCoupling::Step(const MotorSamples& motors, const arm::JointTorques& collision)
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

        dynamics.Step(torques, collision, MotorTimes(motors));
        const auto speeds = dynamics.Speeds();
        const auto accelerations = dynamics.Accelerations();
        const auto inertias = dynamics.ApparentInertias(torques);
        ShaftSpeeds shafts{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (motors[joint].has_value())
            {
                const auto& transmission = transmissions[joint];
                const auto inertia = inertias[joint] / (transmission.ratio * transmission.ratio);
                const auto load = motors[joint]->torque - inertia * transmission.Gain() * accelerations[joint];
                shafts[joint] = plant_port::ShaftDrive{ transmission.MotorSpeed(speeds[joint]).Value(), motors[joint]->period, load, inertia };
            }
        return shafts;
    }

    simulation::StepTimes ArmCoupling::MotorTimes(const MotorSamples& motors)
    {
        simulation::StepTimes times{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto& motor = motors[joint];
            const auto& last = lastPeriods[joint];
            if (motor.has_value() && last.has_value() && motor->period > *last)
                times[joint] = std::min(static_cast<float>(motor->period - *last) * motor->periodSeconds, longestStep);
            lastPeriods[joint] = motor.has_value() ? std::optional{ motor->period } : std::nullopt;
        }
        return times;
    }

    bool ArmCoupling::Engaged() const
    {
        return engaged;
    }
}
