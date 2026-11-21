#pragma once

#include "e-foc-protocol/foc/Units.hpp"
#include "numerical/math/Math.hpp"
#include <cstdint>

namespace arm
{
    enum class JointDirection : int8_t
    {
        positive = 1,
        negative = -1
    };

    struct JointTransmission
    {
        float ratio{ 1.0f };
        JointDirection direction{ JointDirection::positive };
        foc::Radians offset{ 0.0f };

        constexpr float Gain() const
        {
            return direction == JointDirection::positive ? ratio : -ratio;
        }

        foc::Radians MotorPosition(foc::Radians joint) const
        {
            return foc::Radians{ Gain() * (joint.Value() - offset.Value()) };
        }

        foc::Radians JointPosition(foc::Radians motor) const
        {
            return foc::Radians{ motor.Value() / Gain() + offset.Value() };
        }

        foc::RadiansPerSecond MotorSpeed(foc::RadiansPerSecond joint) const
        {
            return foc::RadiansPerSecond{ Gain() * joint.Value() };
        }

        foc::RadiansPerSecond JointSpeed(foc::RadiansPerSecond motor) const
        {
            return foc::RadiansPerSecond{ motor.Value() / Gain() };
        }

        foc::NewtonMeter MotorTorque(foc::NewtonMeter joint) const
        {
            return foc::NewtonMeter{ joint.Value() / Gain() };
        }

        bool Valid() const
        {
            return math::IsFinite(ratio) && ratio > 0.0f && math::IsFinite(offset.Value());
        }
    };
}
