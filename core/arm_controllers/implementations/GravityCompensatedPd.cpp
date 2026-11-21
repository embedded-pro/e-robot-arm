#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"

namespace arm
{
    PdGainMatrices PlacedGains(const DrivenArm& drivenArm, const BandwidthTuning& tuning, const JointPositions& positions)
    {
        const auto inertia = drivenArm.MassMatrix(drivenArm.Model().ModelVector(positions));
        return { inertia * (tuning.bandwidth * tuning.bandwidth), inertia * (2.0f * tuning.dampingRatio * tuning.bandwidth) };
    }

    GravityCompensatedPd::GravityCompensatedPd(const DrivenArm& drivenArm, const BandwidthTuning& tuning)
        : drivenArm(drivenArm)
        , model(drivenArm.Model())
        , tuning(tuning)
    {}

    bool GravityCompensatedPd::BumplessTransfer() const
    {
        return true;
    }

    void GravityCompensatedPd::Reset(const MeasuredJoints&, const JointReference&)
    {}

    JointSetpoints GravityCompensatedPd::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        const auto gains = PlacedGains(drivenArm, tuning, measured.positions);
        const Law law{ model.Dynamics(), gains.kp, gains.kd };
        const auto torques = law.ComputeTorque(model.ModelVector(measured.positions), model.ModelSpeeds(measured.speeds), model.ModelVector(reference.positions));
        return JointSetpoints{ reference.positions, model.ToJointTorques(torques) };
    }
}
