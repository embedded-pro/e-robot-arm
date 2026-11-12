#include "tools/arm_host/DefaultArmConfig.hpp"
#include <numbers>

namespace arm_host
{
    arm::ArmConfig DefaultArmConfig(const arm::JointNodeIds& nodes)
    {
        arm::ArmConfig config{ nodes, 3, {}, {}, std::chrono::milliseconds{ 10 } };
        for (auto& limit : config.positionLimits)
            limit = { foc::Radians{ -std::numbers::pi_v<float> }, foc::Radians{ std::numbers::pi_v<float> } };
        for (auto& limit : config.motionLimits)
            limit = { foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecondSquared{ 2.0f } };
        return config;
    }
}
