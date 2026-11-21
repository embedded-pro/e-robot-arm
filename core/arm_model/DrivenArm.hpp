#ifndef CORE_ARM_MODEL_DRIVEN_ARM_HPP
#define CORE_ARM_MODEL_DRIVEN_ARM_HPP

#include "core/arm_model/ArmModel.hpp"
#include "robotics/dynamics/ChainDynamicsModel.hpp"

namespace arm
{
    // The arm model with each joint's drive: its reflected rotor inertia, the motor's inertia times the gear ratio
    // squared, as the armature of the joint's link. The links' dynamics then carry the rotors: the mass matrix has
    // them on its diagonal and the inverse dynamics adds each rotor's inertia times its joint's acceleration.
    class DrivenArm
    {
    public:
        using Dynamics = dynamics::ChainDynamicsModel<float, jointCount>;

        DrivenArm(const ArmModel& model, const JointInertias& rotors);
        DrivenArm(const DrivenArm& other) = delete;
        DrivenArm& operator=(const DrivenArm& other) = delete;

        const ArmModel& Model() const;
        const JointInertias& Rotors() const;
        const ArmModel::DynamicLinks& Links() const;
        const Dynamics& LinkDynamics() const;

        ArmModel::MassMatrix MassMatrix(const ArmModel::JointVector& angles) const;
        ArmModel::MassMatrix JointMassMatrix(const JointPositions& positions) const;

    private:
        const ArmModel& model;
        JointInertias rotors;
        ArmModel::DynamicLinks links;
        Dynamics dynamics;
    };
}

#endif
