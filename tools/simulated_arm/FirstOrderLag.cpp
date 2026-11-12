#include "tools/simulated_arm/FirstOrderLag.hpp"
#include <chrono>
#include <cmath>

namespace simulation
{
    JointMotionState FollowWithLag(foc::Radians position, foc::Radians target, infra::Duration elapsed, infra::Duration timeConstant)
    {
        if (timeConstant <= infra::Duration::zero())
            return { target, foc::RadiansPerSecond{ 0.0f } };

        const auto tau = std::chrono::duration<float>(timeConstant).count();
        const auto dt = std::chrono::duration<float>(elapsed).count();
        const auto error = position.Value() - target.Value();
        const auto followed = target.Value() + error * std::exp(-dt / tau);

        return { foc::Radians{ followed }, foc::RadiansPerSecond{ (target.Value() - followed) / tau } };
    }
}
