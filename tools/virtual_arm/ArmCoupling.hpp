#pragma once

#include "core/arm/ArmConfig.hpp"
#include "tools/simulated_arm/ArmDynamics.hpp"
#include "tools/virtual_arm/plant_port/PlantPortCodec.hpp"
#include <array>
#include <optional>

namespace virtual_arm
{
    struct MotorSample
    {
        float angle{ 0.0f };
        float torque{ 0.0f };
        uint64_t period{ 0 };
        float periodSeconds{ 0.0f };
    };

    using MotorSamples = std::array<std::optional<MotorSample>, arm::jointCount>;
    using ShaftSpeeds = std::array<std::optional<plant_port::ShaftDrive>, arm::jointCount>;

    class ArmCoupling
    {
    public:
        explicit ArmCoupling(const arm::ArmConfig& config);

        ShaftSpeeds Step(const MotorSamples& motors, const arm::JointTorques& collision);
        bool Engaged() const;

    private:
        simulation::StepTimes MotorTimes(const MotorSamples& motors);

        arm::JointTransmissions transmissions;
        simulation::ArmDynamics dynamics;
        std::array<std::optional<uint64_t>, arm::jointCount> lastPeriods{};
        bool engaged{ false };
    };
}
