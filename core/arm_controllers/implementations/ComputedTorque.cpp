#include "core/arm_controllers/implementations/ComputedTorque.hpp"

namespace arm
{
    namespace
    {
        using GainMatrix = controllers::ComputedTorqueControl<float, jointCount>::GainMatrix;

        GainMatrix Scaled(float gain)
        {
            return GainMatrix::Identity() * gain;
        }
    }

    ComputedTorque::ComputedTorque(const DrivenArm& drivenArm, const BandwidthTuning& tuning)
        : model(drivenArm.Model())
        , law(drivenArm.LinkDynamics(), Scaled(tuning.bandwidth * tuning.bandwidth), Scaled(2.0f * tuning.dampingRatio * tuning.bandwidth))
    {}

    bool ComputedTorque::BumplessTransfer() const
    {
        return true;
    }

    void ComputedTorque::Reset(const MeasuredJoints&, const JointReference&)
    {}

    JointSetpoints ComputedTorque::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        const auto torques = law.ComputeTorque(model.ModelVector(measured.positions), model.ModelSpeeds(measured.speeds), model.ModelVector(reference.positions),
            model.ModelSpeeds(reference.speeds), model.ModelAccelerations(reference.accelerations));
        return JointSetpoints{ reference.positions, model.ToJointTorques(torques) };
    }
}
