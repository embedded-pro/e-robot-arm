#pragma once

#include "infra/timer/Timer.hpp"
#include "tools/simulated_arm/ArmDynamics.hpp"
#include "tools/simulated_arm/JointPlant.hpp"
#include "tools/simulated_arm/SimulatedJoint.hpp"
#include <array>
#include <optional>

namespace simulation
{
    class ArmPlant
        : public JointPlant
    {
    public:
        static constexpr infra::Duration step{ std::chrono::milliseconds{ 1 } };
        static constexpr infra::Duration longestCatchUp{ std::chrono::milliseconds{ 100 } };

        ArmPlant(const PlantConfig& config, const std::array<SimulatedJoint*, arm::jointCount>& joints);

        void Advance() override;
        foc::Radians MotorPosition(std::size_t joint) const override;
        foc::RadiansPerSecond MotorSpeed(std::size_t joint) const override;

    private:
        void TakeKinematicJoints(const DrivingTorques& torques);

        arm::JointTransmissions transmissions;
        std::array<SimulatedJoint*, arm::jointCount> joints;
        ArmDynamics dynamics;
        infra::TimePoint simulated{ infra::Now() };
    };
}
