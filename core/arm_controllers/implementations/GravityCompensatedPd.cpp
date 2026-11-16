#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"

namespace arm
{
    GravityCompensatedPd::GravityCompensatedPd(const ArmModel& model, const JointPdGains& gains)
        : model(model)
        , law(model.Dynamics(), Diagonal(gains, &PdGain::kp), Diagonal(gains, &PdGain::kd))
    {}

    bool GravityCompensatedPd::BumplessTransfer() const
    {
        return true;
    }

    can::FocMotorMode GravityCompensatedPd::Mode() const
    {
        return can::FocMotorMode::torque;
    }

    void GravityCompensatedPd::Reset(const MeasuredJoints&)
    {}

    JointSetpoints GravityCompensatedPd::Step(const MeasuredJoints& measured, const JointPositions& reference)
    {
        const auto torques = law.ComputeTorque(model.ModelVector(measured.positions), model.ModelSpeeds(measured.speeds), model.ModelVector(reference));
        return JointSetpoints{ reference, model.ToJointTorques(torques) };
    }

    GravityCompensatedPd::Law::GainMatrix GravityCompensatedPd::Diagonal(const JointPdGains& gains, float PdGain::*gain)
    {
        Law::GainMatrix matrix{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            matrix.at(joint, joint) = gains[joint].*gain;
        return matrix;
    }
}
