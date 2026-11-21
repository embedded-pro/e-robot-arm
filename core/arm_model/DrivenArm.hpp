#pragma once

#include "core/arm_model/ArmModel.hpp"
#include "robotics/dynamics/ChainDynamicsModel.hpp"

namespace arm
{
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
