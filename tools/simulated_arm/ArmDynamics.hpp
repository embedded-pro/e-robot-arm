#pragma once

#include "core/arm_model/ArmModel.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/joint_configurator/interfaces/MotorSettings.hpp"
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
        std::array<float, arm::jointCount> coulombFriction{};
    };

    PlantConfig PlantFor(const arm::JointTransmissions& transmissions, const arm::ArmGeometry& geometry, const arm::ArmMass& mass, const arm::JointMotors& motors);

    using DrivingTorques = std::array<std::optional<foc::NewtonMeter>, arm::jointCount>;
    using StepTimes = std::array<float, arm::jointCount>;

    class ArmDynamics
    {
    public:
        explicit ArmDynamics(const PlantConfig& config);

        void SetState(const arm::JointPositions& positions, const arm::JointSpeeds& speeds);
        void SetPositions(const arm::JointPositions& positions);
        void Step(const DrivingTorques& torques, const arm::JointTorques& external, float seconds);
        void Step(const DrivingTorques& torques, const arm::JointTorques& external, const StepTimes& seconds);

        arm::JointPositions Positions() const;
        arm::JointSpeeds Speeds() const;
        std::array<float, arm::jointCount> Accelerations() const;
        std::array<float, arm::jointCount> ApparentInertias(const DrivingTorques& torques) const;

    private:
        using JointVector = arm::ArmModel::JointVector;
        using MassMatrix = arm::ArmModel::MassMatrix;

        struct System
        {
            MassMatrix mass;
            JointVector force;
        };

        System Assemble(const DrivingTorques& torques, const arm::JointTorques& external) const;

        JointVector DrivingForce(const DrivingTorques& torques, const arm::JointTorques& external) const;
        void StopReversedJoints(const JointVector& before);

        arm::JointTransmissions transmissions;
        std::array<float, arm::jointCount> armature{};
        std::array<float, arm::jointCount> friction{};
        std::array<float, arm::jointCount> coulomb{};
        arm::ArmModel model;
        JointVector q{};
        JointVector qDot{};
        JointVector qDDot{};
    };
}
