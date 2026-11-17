#include "tools/simulated_arm/ArmPlant.hpp"
#include "numerical/math/CholeskyDecomposition.hpp"
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
        constexpr float stepSeconds{ std::chrono::duration<float>(ArmPlant::step).count() };

        arm::JointLimits Unlimited()
        {
            arm::JointLimits limits{};
            for (auto& limit : limits)
                limit = { foc::Radians{ -1.0e3f }, foc::Radians{ 1.0e3f } };
            return limits;
        }
    }

    ArmPlant::ArmPlant(const PlantConfig& config, const std::array<SimulatedJoint*, arm::jointCount>& joints)
        : transmissions(config.transmissions)
        , joints(joints)
        , model(config.geometry, config.mass, Unlimited())
    {
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto ratio = transmissions[joint].ratio;
            armature[joint] = config.rotorInertia[joint] * ratio * ratio;
            friction[joint] = config.motorFriction[joint] * ratio * ratio;
        }
        arm::JointPositions positions{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            positions[joint] = transmissions[joint].JointPosition(joints[joint]->Position());
        q = model.ModelVector(positions);
    }

    void ArmPlant::Advance()
    {
        const auto now = infra::Now();
        if (simulated + step > now)
            return;

        Torques torques{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            torques[joint] = joints[joint]->DrivingTorque();

        TakeKinematicJoints(torques);
        const auto driven = std::ranges::any_of(torques, [](const std::optional<foc::NewtonMeter>& torque)
            {
                return torque.has_value();
            });
        if (!driven || now - simulated > longestCatchUp)
            simulated = driven ? now - longestCatchUp : now;
        for (; simulated + step <= now; simulated += step)
            Step(torques);
    }

    foc::Radians ArmPlant::MotorPosition(std::size_t joint) const
    {
        return transmissions[joint].MotorPosition(model.ToJointPositions(q)[joint]);
    }

    foc::RadiansPerSecond ArmPlant::MotorSpeed(std::size_t joint) const
    {
        return transmissions[joint].MotorSpeed(model.ToJointSpeeds(qDot)[joint]);
    }

    void ArmPlant::Step(const Torques& torques)
    {
        const auto& dynamics = model.Dynamics();
        auto mass = dynamics.ComputeMassMatrix(q);
        JointVector force{ ModelTorques(torques) - dynamics.ComputeCoriolisTerms(q, qDot) - dynamics.ComputeGravityTerms(q) };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (torques[joint].has_value())
                mass.at(joint, joint) += armature[joint];
            else
            {
                for (std::size_t other = 0; other != arm::jointCount; ++other)
                {
                    mass.at(joint, other) = 0.0f;
                    mass.at(other, joint) = 0.0f;
                }
                mass.at(joint, joint) = 1.0f;
                force.at(joint, 0) = 0.0f;
            }

        const auto acceleration = math::CholeskyDecomposition<float, arm::jointCount>::Solve(mass, force);
        if (!acceleration.has_value())
            return;

        qDot = JointVector{ qDot + *acceleration * stepSeconds };
        q = JointVector{ q + qDot * stepSeconds };
    }

    void ArmPlant::TakeKinematicJoints(const Torques& torques)
    {
        auto positions = model.ToJointPositions(q);
        auto speeds = model.ToJointSpeeds(qDot);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (!torques[joint].has_value())
            {
                joints[joint]->Settle();
                positions[joint] = transmissions[joint].JointPosition(joints[joint]->Position());
                speeds[joint] = foc::RadiansPerSecond{ 0.0f };
            }
        q = model.ModelVector(positions);
        qDot = model.ModelSpeeds(speeds);
    }

    ArmPlant::JointVector ArmPlant::ModelTorques(const Torques& torques) const
    {
        const auto speeds = model.ToJointSpeeds(qDot);
        arm::JointTorques joint{};
        for (std::size_t index = 0; index != arm::jointCount; ++index)
            if (torques[index].has_value())
                joint[index] = foc::NewtonMeter{ transmissions[index].Gain() * torques[index]->Value() - friction[index] * speeds[index].Value() };
        return model.ModelTorques(joint);
    }
}
