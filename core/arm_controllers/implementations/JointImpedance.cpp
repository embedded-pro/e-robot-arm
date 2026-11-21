#include "core/arm_controllers/implementations/JointImpedance.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/solvers/GaussianElimination.hpp"

namespace arm
{
    JointStiffnesses ImpedanceStiffness(const ArmModel::MassMatrix& inertia, const JointImpedanceTuning& tuning)
    {
        if (tuning.stiffness.has_value())
            return *tuning.stiffness;

        const auto mobility = solvers::SolveSystem(inertia, ArmModel::MassMatrix::Identity());
        JointStiffnesses stiffness{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            stiffness[joint] = defaultImpedanceBandwidth * defaultImpedanceBandwidth / mobility.at(joint, joint);
        return stiffness;
    }

    ArmModel::MassMatrix CoupledDamping(const ArmModel::MassMatrix& inertia, const JointStiffnesses& stiffness, float dampingRatio)
    {
        std::array<float, jointCount> weight{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            weight[joint] = inertia.at(joint, joint) > 0.0f ? math::Sqrt(math::Sqrt(stiffness[joint] / inertia.at(joint, joint))) : 0.0f;

        const auto w = Diagonal(weight);
        return w * inertia * w * (2.0f * dampingRatio);
    }

    JointTorques Feedforward(const ArmModel& model, const MeasuredJoints& measured, const JointReference& reference)
    {
        return model.InverseDynamics(measured.positions, reference.speeds, reference.accelerations);
    }

    JointImpedance::JointImpedance(const DrivenArm& drivenArm, const JointImpedanceTuning& tuning)
        : drivenArm(drivenArm)
        , model(drivenArm.Model())
        , profileStiffness(ImpedanceStiffness(drivenArm.JointMassMatrix(JointPositions{}), tuning))
        , dampingRatio(tuning.dampingRatio)
    {
        Size();
    }

    bool JointImpedance::SetStiffnessShare(float newShare)
    {
        if (!(newShare >= minimumStiffnessShare && newShare <= 1.0f))
            return false;

        share = newShare;
        Size();
        return true;
    }

    float JointImpedance::StiffnessShare() const
    {
        return share;
    }

    ArmModel::MassMatrix JointImpedance::Damping(const JointPositions& positions) const
    {
        return CoupledDamping(drivenArm.JointMassMatrix(positions), stiffness, dampingRatio);
    }

    bool JointImpedance::BumplessTransfer() const
    {
        return true;
    }

    void JointImpedance::Reset(const MeasuredJoints&, const JointReference&)
    {}

    JointSetpoints JointImpedance::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        const JointVector positionError{ ToVector(reference.positions) - ToVector(measured.positions) };
        const JointVector speedError{ ToVector(reference.speeds) - ToVector(measured.speeds) };
        const JointVector torques{ ToVector(Feedforward(model, measured, reference)) + Diagonal(stiffness) * positionError +
                                   Damping(measured.positions) * speedError };
        return JointSetpoints{ reference.positions, FromVector<foc::NewtonMeter>(torques) };
    }

    void JointImpedance::Size()
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            stiffness[joint] = share * profileStiffness[joint];
    }
}
