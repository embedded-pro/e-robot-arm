#include "core/arm_model/DrivenArm.hpp"

namespace arm
{
    namespace
    {
        ArmModel::DynamicLinks WithRotors(const ArmModel& model, const JointInertias& rotors)
        {
            auto links = model.Links();
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                links[joint].armature = rotors[joint];
            return links;
        }
    }

    DrivenArm::DrivenArm(const ArmModel& model, const JointInertias& rotors)
        : model(model)
        , rotors(rotors)
        , links(WithRotors(model, rotors))
        , dynamics(links, model.Gravity())
    {}

    const ArmModel& DrivenArm::Model() const
    {
        return model;
    }

    const JointInertias& DrivenArm::Rotors() const
    {
        return rotors;
    }

    const ArmModel::DynamicLinks& DrivenArm::Links() const
    {
        return links;
    }

    const DrivenArm::Dynamics& DrivenArm::LinkDynamics() const
    {
        return dynamics;
    }

    ArmModel::MassMatrix DrivenArm::MassMatrix(const ArmModel::JointVector& angles) const
    {
        return dynamics.ComputeMassMatrix(angles);
    }

    ArmModel::MassMatrix DrivenArm::JointMassMatrix(const JointPositions& positions) const
    {
        auto inertia = model.JointMassMatrix(positions);
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            inertia.at(joint, joint) += rotors[joint];
        return inertia;
    }
}
