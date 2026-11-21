#include "tools/simulated_arm/ArmDynamics.hpp"
#include "core/arm_model/JointFrameMap.hpp"
#include "numerical/math/CholeskyDecomposition.hpp"
#include "robotics/dynamics/EffectiveJointInertia.hpp"
#include <algorithm>

namespace simulation
{
    PlantConfig PlantFor(const arm::JointTransmissions& transmissions, const arm::ArmGeometry& geometry, const arm::ArmMass& mass, const arm::JointMotors& motors)
    {
        PlantConfig plant{ transmissions, geometry, mass };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto& motor = motors[joint].has_value() ? motors[joint]->settings : arm::motorPresets[0].settings;
            plant.rotorInertia[joint] = motor[arm::MotorParameter::inertia];
            plant.motorFriction[joint] = motor[arm::MotorParameter::viscousFriction];
        }
        return plant;
    }

    namespace
    {
        arm::JointLimits Unlimited()
        {
            arm::JointLimits limits{};
            for (auto& limit : limits)
                limit = { foc::Radians{ -1.0e3f }, foc::Radians{ 1.0e3f } };
            return limits;
        }

        float Sign(float value)
        {
            if (value > 0.0f)
                return 1.0f;
            if (value < 0.0f)
                return -1.0f;
            return 0.0f;
        }
    }

    ArmDynamics::ArmDynamics(const PlantConfig& config)
        : transmissions(config.transmissions)
        , model(config.geometry, config.mass, Unlimited())
    {
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto ratio = transmissions[joint].ratio;
            armature[joint] = config.rotorInertia[joint] * ratio * ratio;
            friction[joint] = config.motorFriction[joint] * ratio * ratio;
            coulomb[joint] = config.coulombFriction[joint] * ratio;
        }
    }

    void ArmDynamics::SetState(const arm::JointPositions& positions, const arm::JointSpeeds& speeds)
    {
        q = model.ModelVector(positions);
        qDot = model.ModelSpeeds(speeds);
    }

    void ArmDynamics::SetPositions(const arm::JointPositions& positions)
    {
        q = model.ModelVector(positions);
    }

    void ArmDynamics::Step(const DrivingTorques& torques, const arm::JointTorques& external, float seconds)
    {
        StepTimes times{};
        times.fill(seconds);
        Step(torques, external, times);
    }

    void ArmDynamics::Step(const DrivingTorques& torques, const arm::JointTorques& external, const StepTimes& seconds)
    {
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (!torques[joint].has_value())
                qDot.at(joint, 0) = 0.0f;

        const auto system = Assemble(torques, external);
        const auto acceleration = math::CholeskyDecomposition<float, arm::jointCount>::Solve(system.mass, system.force);
        if (!acceleration.has_value())
        {
            qDDot = JointVector{};
            return;
        }

        qDDot = *acceleration;
        const auto times = math::Diagonal(arm::ToVector(seconds));
        const auto before = qDot;
        qDot = JointVector{ qDot + times * *acceleration };
        StopReversedJoints(before);
        q = JointVector{ q + times * qDot };
    }

    std::array<float, arm::jointCount> ArmDynamics::ApparentInertias(const DrivingTorques& torques) const
    {
        return arm::ToValues(dynamics::EffectiveJointInertia(Assemble(torques, arm::JointTorques{}).mass).value_or(JointVector{}));
    }

    ArmDynamics::System ArmDynamics::Assemble(const DrivingTorques& torques, const arm::JointTorques& external) const
    {
        const auto& dynamics = model.Dynamics();
        System system{ dynamics.ComputeMassMatrix(q), JointVector{ DrivingForce(torques, external) - dynamics.ComputeCoriolisTerms(q, qDot) - dynamics.ComputeGravityTerms(q) } };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (torques[joint].has_value())
            {
                system.mass.at(joint, joint) += armature[joint];
                const auto speed = qDot.at(joint, 0);
                const auto drag = speed != 0.0f ? coulomb[joint] * Sign(speed) : std::clamp(system.force.at(joint, 0), -coulomb[joint], coulomb[joint]);
                system.force.at(joint, 0) -= drag;
            }
            else
            {
                for (std::size_t other = 0; other != arm::jointCount; ++other)
                {
                    system.mass.at(joint, other) = 0.0f;
                    system.mass.at(other, joint) = 0.0f;
                }
                system.mass.at(joint, joint) = 1.0f;
                system.force.at(joint, 0) = 0.0f;
            }
        return system;
    }

    arm::JointPositions ArmDynamics::Positions() const
    {
        return model.ToJointPositions(q);
    }

    arm::JointSpeeds ArmDynamics::Speeds() const
    {
        return model.ToJointSpeeds(qDot);
    }

    std::array<float, arm::jointCount> ArmDynamics::Accelerations() const
    {
        std::array<float, arm::jointCount> accelerations{};
        std::ranges::transform(model.ToJointSpeeds(qDDot), accelerations.begin(), [](foc::RadiansPerSecond acceleration)
            {
                return acceleration.Value();
            });
        return accelerations;
    }

    ArmDynamics::JointVector ArmDynamics::DrivingForce(const DrivingTorques& torques, const arm::JointTorques& external) const
    {
        const auto speeds = model.ToJointSpeeds(qDot);
        arm::JointTorques joint{};
        for (std::size_t index = 0; index != arm::jointCount; ++index)
            if (torques[index].has_value())
                joint[index] = foc::NewtonMeter{ transmissions[index].Gain() * torques[index]->Value() - friction[index] * speeds[index].Value() +
                                                 external[index].Value() };
        return model.ModelTorques(joint);
    }

    void ArmDynamics::StopReversedJoints(const JointVector& before)
    {
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (coulomb[joint] > 0.0f && before.at(joint, 0) * qDot.at(joint, 0) < 0.0f)
                qDot.at(joint, 0) = 0.0f;
    }
}
