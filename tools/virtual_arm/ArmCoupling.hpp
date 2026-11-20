#ifndef TOOLS_VIRTUAL_ARM_ARM_COUPLING_HPP
#define TOOLS_VIRTUAL_ARM_ARM_COUPLING_HPP

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
    };

    using MotorSamples = std::array<std::optional<MotorSample>, arm::jointCount>;
    using ShaftSpeeds = std::array<std::optional<plant_port::ShaftDrive>, arm::jointCount>;

    class ArmCoupling
    {
    public:
        explicit ArmCoupling(const arm::ArmConfig& config);

        ShaftSpeeds Step(const MotorSamples& motors, const arm::JointTorques& collision, float seconds);
        bool Engaged() const;

    private:
        arm::JointTransmissions transmissions;
        simulation::ArmDynamics dynamics;
        bool engaged{ false };
    };
}

#endif
