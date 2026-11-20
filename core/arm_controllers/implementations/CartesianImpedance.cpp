#include "core/arm_controllers/implementations/CartesianImpedance.hpp"
#include "core/arm_controllers/implementations/JointImpedance.hpp"
#include "robotics/controllers/manipulator/TaskError.hpp"

namespace arm
{
    CartesianImpedance::CartesianImpedance(const ArmModel& model, const JointInertias& rotors, const CartesianImpedanceTuning& tuning)
        : model(model)
        , provider(model)
        , rotors(rotors)
        , tuning(tuning)
    {
        Size();
    }

    bool CartesianImpedance::SetStiffnessShare(float newShare)
    {
        if (!(newShare >= minimumStiffnessShare && newShare <= 1.0f))
            return false;

        share = newShare;
        Size();
        return true;
    }

    float CartesianImpedance::StiffnessShare() const
    {
        return share;
    }

    ArmModel::MassMatrix CartesianImpedance::Damping(const ArmModel::JointVector& angles) const
    {
        auto inertia = model.Dynamics().ComputeMassMatrix(angles);
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            inertia.at(joint, joint) += rotors[joint];

        const auto jacobian = provider.Jacobian(angles);
        JointStiffnesses jointStiffness{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            for (std::size_t axis = 0; axis != 6; ++axis)
                jointStiffness[joint] += stiffness[axis] * jacobian.at(axis, joint) * jacobian.at(axis, joint);

        return CoupledDamping(inertia, jointStiffness, tuning.dampingRatio);
    }

    bool CartesianImpedance::BumplessTransfer() const
    {
        return true;
    }

    void CartesianImpedance::Reset(const MeasuredJoints&, const JointReference&)
    {}

    JointSetpoints CartesianImpedance::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        const auto angles = model.ModelVector(measured.positions);
        const auto speedError = model.ModelSpeeds(reference.speeds) - model.ModelSpeeds(measured.speeds);
        const auto error = controllers::TaskError<float, 6>(provider.ToolPose(model.ModelVector(reference.positions)), provider.ToolPose(angles));

        math::Vector<float, 6> wrench{};
        for (std::size_t axis = 0; axis != 6; ++axis)
            wrench.at(axis, 0) = stiffness[axis] * error.at(axis, 0);

        const ArmModel::JointVector torques{ provider.Jacobian(angles).Transpose() * wrench + Damping(angles) * speedError };
        auto result = Feedforward(model, measured, reference);
        const auto feedback = model.ToJointTorques(torques);
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            result[joint] = foc::NewtonMeter{ result[joint].Value() + feedback[joint].Value() };
        return JointSetpoints{ reference.positions, result };
    }

    void CartesianImpedance::Size()
    {
        for (std::size_t axis = 0; axis != 3; ++axis)
        {
            stiffness[axis] = share * tuning.translationalStiffness;
            stiffness[axis + 3] = share * tuning.rotationalStiffness;
        }
    }
}
