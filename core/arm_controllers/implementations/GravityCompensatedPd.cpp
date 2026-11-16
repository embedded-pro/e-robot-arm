#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"

namespace arm
{
    namespace
    {
        using GainMatrix = controllers::PdGravityCompensation<float, jointCount>::GainMatrix;

        GainMatrix Stiffness(const JointPdGains& gains)
        {
            GainMatrix matrix{};
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                matrix.at(joint, joint) = gains[joint].kp;
            return matrix;
        }

        GainMatrix Damping(const JointPdGains& gains)
        {
            GainMatrix matrix{};
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                matrix.at(joint, joint) = gains[joint].kd;
            return matrix;
        }
    }

    GravityCompensatedPd::GravityCompensatedPd(const ArmModel& model, const JointPdGains& gains)
        : model(model)
        , law(model.Dynamics(), Stiffness(gains), Damping(gains))
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
}
