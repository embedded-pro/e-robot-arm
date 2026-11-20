#include "core/arm_controllers/implementations/JointImpedance.hpp"
#include "numerical/math/Math.hpp"

namespace arm
{
    JointInertias HomeInertias(const ArmModel& model, const JointInertias& rotors)
    {
        const auto inertia = model.Dynamics().ComputeMassMatrix(model.ModelVector(JointPositions{}));
        JointInertias home{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            home[joint] = inertia.at(joint, joint) + rotors[joint];
        return home;
    }

    JointStiffnesses ImpedanceStiffness(const JointInertias& inertias, const JointImpedanceTuning& tuning)
    {
        if (tuning.stiffness.has_value())
            return *tuning.stiffness;

        JointStiffnesses stiffness{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            stiffness[joint] = defaultImpedanceBandwidth * defaultImpedanceBandwidth * inertias[joint];
        return stiffness;
    }

    JointImpedance::JointImpedance(const ArmModel& model, const JointInertias& inertias, const JointImpedanceTuning& tuning)
        : model(model)
        , inertias(inertias)
        , profileStiffness(ImpedanceStiffness(inertias, tuning))
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

    bool JointImpedance::BumplessTransfer() const
    {
        return true;
    }

    void JointImpedance::Reset(const MeasuredJoints&, const JointReference&)
    {}

    JointSetpoints JointImpedance::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        auto torques = model.GravityTorque(measured.positions);
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto spring = stiffness[joint] * (reference.positions[joint].Value() - measured.positions[joint].Value());
            const auto damper = damping[joint] * (reference.speeds[joint].Value() - measured.speeds[joint].Value());
            torques[joint] = foc::NewtonMeter{ torques[joint].Value() + spring + damper };
        }
        return JointSetpoints{ reference.positions, torques };
    }

    void JointImpedance::Size()
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            stiffness[joint] = share * profileStiffness[joint];
            damping[joint] = 2.0f * dampingRatio * math::Sqrt(stiffness[joint] * inertias[joint]);
        }
    }
}
