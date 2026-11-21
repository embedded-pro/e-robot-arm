#include "core/arm_controllers/implementations/OperationalSpace.hpp"
#include <algorithm>

namespace arm
{
    namespace
    {
        math::SquareMatrix<float, 6> Gain(float value)
        {
            return math::SquareMatrix<float, 6>::Identity() * value;
        }
    }

    OperationalSpace::OperationalSpace(const DrivenArm& drivenArm, const OperationalSpaceTuning& tuning, float manipulabilityMin)
        : model(drivenArm.Model())
        , provider(model)
        , dynamics(drivenArm.LinkDynamics())
        , law(dynamics, provider, Gain(tuning.bandwidth * tuning.bandwidth), Gain(2.0f * tuning.dampingRatio * tuning.bandwidth), tuning.singularityDamping)
        , stiffness(tuning.bandwidth * tuning.bandwidth)
        , damping(2.0f * tuning.dampingRatio * tuning.bandwidth)
        , manipulabilityMin(manipulabilityMin)
    {}

    float OperationalSpace::JointSpaceShare(const JointPositions& positions) const
    {
        if (!(manipulabilityMin > 0.0f))
            return 0.0f;
        return std::clamp((manipulabilityMin - model.Manipulability(positions)) / (fullJointSpaceBelow * manipulabilityMin), 0.0f, 1.0f);
    }

    bool OperationalSpace::BumplessTransfer() const
    {
        return true;
    }

    void OperationalSpace::Reset(const MeasuredJoints&, const JointReference&)
    {}

    JointSetpoints OperationalSpace::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        const auto angles = model.ModelVector(measured.positions);
        const auto speeds = model.ModelSpeeds(measured.speeds);
        const auto share = JointSpaceShare(measured.positions);

        ArmModel::JointVector torques{};
        if (share < 1.0f)
        {
            const auto target = model.ModelVector(reference.positions);
            const auto targetSpeeds = model.ModelSpeeds(reference.speeds);
            const auto targetJacobian = provider.Jacobian(target);
            const math::Vector<float, 6> twist{ targetJacobian * targetSpeeds };
            const math::Vector<float, 6> acceleration{ targetJacobian * model.ModelAccelerations(reference.accelerations) + provider.BiasAcceleration(target, targetSpeeds) };
            torques = law.ComputeTorque(angles, speeds, provider.ToolPose(target), twist, acceleration, ArmModel::JointVector{}) * (1.0f - share);
        }
        if (share > 0.0f)
            torques = torques + JointSpaceTorques(angles, speeds, reference) * share;

        return JointSetpoints{ reference.positions, model.ToJointTorques(torques) };
    }

    ArmModel::JointVector OperationalSpace::JointSpaceTorques(const ArmModel::JointVector& angles, const ArmModel::JointVector& speeds, const JointReference& reference) const
    {
        const ArmModel::JointVector acceleration{ model.ModelAccelerations(reference.accelerations) + (model.ModelVector(reference.positions) - angles) * stiffness +
                                                  (model.ModelSpeeds(reference.speeds) - speeds) * damping };
        return dynamics.ComputeMassMatrix(angles) * acceleration + dynamics.ComputeCoriolisTerms(angles, speeds) + dynamics.ComputeGravityTerms(angles);
    }
}
