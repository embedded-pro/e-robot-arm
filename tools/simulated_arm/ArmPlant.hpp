#ifndef TOOLS_SIMULATED_ARM_ARM_PLANT_HPP
#define TOOLS_SIMULATED_ARM_ARM_PLANT_HPP

#include "core/arm_model/ArmModel.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "infra/timer/Timer.hpp"
#include "tools/simulated_arm/JointPlant.hpp"
#include "tools/simulated_arm/SimulatedJoint.hpp"
#include <array>
#include <optional>

namespace simulation
{
    struct PlantConfig
    {
        arm::JointTransmissions transmissions{};
        arm::ArmGeometry geometry{ arm::ReferenceGeometry() };
        arm::ArmMass mass{ arm::ReferenceMass() };
        std::array<float, arm::jointCount> rotorInertia{};
        std::array<float, arm::jointCount> motorFriction{};
    };

    class ArmPlant
        : public JointPlant
    {
    public:
        static constexpr infra::Duration step{ std::chrono::milliseconds{ 1 } };

        ArmPlant(const PlantConfig& config, const std::array<SimulatedJoint*, arm::jointCount>& joints);

        void Advance() override;
        foc::Radians MotorPosition(std::size_t joint) const override;
        foc::RadiansPerSecond MotorSpeed(std::size_t joint) const override;

    private:
        using JointVector = arm::ArmModel::JointVector;
        using Torques = std::array<std::optional<foc::NewtonMeter>, arm::jointCount>;

        void Step(const Torques& torques);
        void TakeKinematicJoints(const Torques& torques);
        JointVector ModelTorques(const Torques& torques) const;

        arm::JointTransmissions transmissions;
        std::array<float, arm::jointCount> armature{};
        std::array<float, arm::jointCount> friction{};
        std::array<SimulatedJoint*, arm::jointCount> joints;
        arm::ArmModel model;
        JointVector q{};
        JointVector qDot{};
        infra::TimePoint simulated{ infra::Now() };
    };
}

#endif
