#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"

namespace arm
{
    PdGainMatrices PlacedGains(const ArmModel& model, const JointInertias& rotors, const BandwidthTuning& tuning, const JointPositions& positions)
    {
        auto inertia = model.Dynamics().ComputeMassMatrix(model.ModelVector(positions));
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            inertia.at(joint, joint) += rotors[joint];

        return { inertia * (tuning.bandwidth * tuning.bandwidth), inertia * (2.0f * tuning.dampingRatio * tuning.bandwidth) };
    }

    GravityCompensatedPd::GravityCompensatedPd(const ArmModel& model, const JointInertias& rotors, const BandwidthTuning& tuning)
        : model(model)
        , rotors(rotors)
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
        const auto gains = PlacedGains(model, rotors, tuning, measured.positions);
        const Law law{ model.Dynamics(), gains.kp, gains.kd };
        const auto torques = law.ComputeTorque(model.ModelVector(measured.positions), model.ModelSpeeds(measured.speeds), model.ModelVector(reference.positions));
        return JointSetpoints{ reference.positions, model.ToJointTorques(torques) };
    }
}
