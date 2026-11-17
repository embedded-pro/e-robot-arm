#include "core/arm_controllers/implementations/ComputedTorque.hpp"

namespace arm
{
    namespace
    {
        using GainMatrix = controllers::ComputedTorqueControl<float, jointCount>::GainMatrix;

        GainMatrix Scaled(float gain)
        {
            GainMatrix matrix{};
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                matrix.at(joint, joint) = gain;
            return matrix;
        }
    }

    ArmatureDynamics::ArmatureDynamics(const dynamics::InverseDynamicsModel<float, jointCount>& arm, const JointInertias& rotors)
        : arm(arm)
        , rotors(rotors)
    {}

    ArmatureDynamics::StateVector ArmatureDynamics::ComputeInverseDynamics(const StateVector& q, const StateVector& qDot, const StateVector& qDDot) const
    {
        auto torques = arm.ComputeInverseDynamics(q, qDot, qDDot);
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            torques.at(joint, 0) += rotors[joint] * qDDot.at(joint, 0);
        return torques;
    }

    ComputedTorque::ComputedTorque(const ArmModel& model, const JointInertias& rotors, const BandwidthTuning& tuning)
        : model(model)
        , dynamics(model.InverseModel(), rotors)
        , law(dynamics, Scaled(tuning.bandwidth * tuning.bandwidth), Scaled(2.0f * tuning.dampingRatio * tuning.bandwidth))
    {}

    bool ComputedTorque::BumplessTransfer() const
    {
        return true;
    }

    void ComputedTorque::Reset(const MeasuredJoints&)
    {}

    JointSetpoints ComputedTorque::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        const auto torques = law.ComputeTorque(model.ModelVector(measured.positions), model.ModelSpeeds(measured.speeds), model.ModelVector(reference.positions),
            model.ModelSpeeds(reference.speeds), model.ModelAccelerations(reference.accelerations));
        return JointSetpoints{ reference.positions, model.ToJointTorques(torques) };
    }
}
