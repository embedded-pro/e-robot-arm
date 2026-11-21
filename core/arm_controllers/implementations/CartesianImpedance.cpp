#include "core/arm_controllers/implementations/CartesianImpedance.hpp"
#include "core/arm_controllers/implementations/JointImpedance.hpp"
#include "robotics/controllers/manipulator/TaskError.hpp"

namespace arm
{
    CartesianImpedance::CartesianImpedance(const DrivenArm& drivenArm, const CartesianImpedanceTuning& tuning)
        : drivenArm(drivenArm)
        , model(drivenArm.Model())
        , provider(model)
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
        const auto inertia = drivenArm.MassMatrix(angles);
        const auto jacobian = provider.Jacobian(angles);
        const auto jointStiffness = DiagonalEntries(jacobian.Transpose() * Diagonal(stiffness) * jacobian);

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

        const math::Vector<float, 6> wrench{ Diagonal(stiffness) * error };
        const ArmModel::JointVector torques{ provider.Jacobian(angles).Transpose() * wrench + Damping(angles) * speedError };
        const JointVector result{ ToVector(Feedforward(model, measured, reference)) + ToVector(model.ToJointTorques(torques)) };
        return JointSetpoints{ reference.positions, FromVector<foc::NewtonMeter>(result) };
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
