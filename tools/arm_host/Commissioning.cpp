#include "tools/arm_host/Commissioning.hpp"

namespace arm_host
{
    namespace
    {
        constexpr float motionShare{ 0.25f };
        constexpr float tripShare{ 0.5f };
    }

    arm::ArmConfig CommissioningConfig(const arm::ArmConfig& profile)
    {
        auto config = profile;
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            auto& motion = config.motionLimits[joint];
            motion = { foc::RadiansPerSecond{ motion.maxSpeed.Value() * motionShare }, foc::RadiansPerSecondSquared{ motion.maxAcceleration.Value() * motionShare } };
            config.collision[joint] = foc::NewtonMeter{ config.collision[joint].Value() * tripShare };
            if (auto& speedTrip = config.speedTrips[joint]; speedTrip.has_value())
                speedTrip = foc::RadiansPerSecond{ speedTrip->Value() * motionShare };
            auto& tracking = config.tracking[joint];
            tracking = { foc::Radians{ tracking.window.Value() * tripShare }, std::chrono::duration_cast<infra::Duration>(tracking.time * tripShare) };
        }
        return config;
    }
}
