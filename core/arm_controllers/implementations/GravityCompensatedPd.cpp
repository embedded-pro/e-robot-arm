#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"

namespace arm
{
    PdGainMatrices DiagonalGains(const JointPdGains& gains)
    {
        PdGainMatrices matrices{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            matrices.kp.at(joint, joint) = gains[joint].kp;
            matrices.kd.at(joint, joint) = gains[joint].kd;
        }
        return matrices;
    }

    PdGainMatrices PlacedGains(const ArmModel& model, const JointInertias& rotors, const BandwidthTuning& tuning)
    {
        auto inertia = model.Dynamics().ComputeMassMatrix(model.ModelVector(JointPositions{}));
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            inertia.at(joint, joint) += rotors[joint];

        return { inertia * (tuning.bandwidth * tuning.bandwidth), inertia * (2.0f * tuning.dampingRatio * tuning.bandwidth) };
    }

    GravityCompensatedPd::GravityCompensatedPd(const ArmModel& model, const PdGainMatrices& gains)
        : model(model)
        , law(model.Dynamics(), gains.kp, gains.kd)
    {}

    bool GravityCompensatedPd::BumplessTransfer() const
    {
        return true;
    }

    void GravityCompensatedPd::Reset(const MeasuredJoints&)
    {}

    JointSetpoints GravityCompensatedPd::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        const auto torques = law.ComputeTorque(model.ModelVector(measured.positions), model.ModelSpeeds(measured.speeds), model.ModelVector(reference.positions));
        return JointSetpoints{ reference.positions, model.ToJointTorques(torques) };
    }
}
