#ifndef TOOLS_SIMULATED_ARM_JOINT_PLANT_HPP
#define TOOLS_SIMULATED_ARM_JOINT_PLANT_HPP

#include "e-foc-protocol/foc/Units.hpp"
#include <cstddef>

namespace simulation
{
    class JointPlant
    {
    public:
        virtual void Advance() = 0;
        virtual foc::Radians MotorPosition(std::size_t joint) const = 0;
        virtual foc::RadiansPerSecond MotorSpeed(std::size_t joint) const = 0;

    protected:
        ~JointPlant() = default;
    };
}

#endif
