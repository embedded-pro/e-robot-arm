#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_JOINT_IMPEDANCE_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_JOINT_IMPEDANCE_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/DrivenArm.hpp"
#include <array>
#include <optional>

namespace arm
{
    using JointStiffnesses = std::array<float, jointCount>;

    inline constexpr float defaultImpedanceBandwidth{ 10.0f };
    inline constexpr float minimumStiffnessShare{ 0.1f };

    struct JointImpedanceTuning
    {
        std::optional<JointStiffnesses> stiffness;
        float dampingRatio{ 0.7f };
    };

    JointStiffnesses ImpedanceStiffness(const ArmModel::MassMatrix& inertia, const JointImpedanceTuning& tuning);
    ArmModel::MassMatrix CoupledDamping(const ArmModel::MassMatrix& inertia, const JointStiffnesses& stiffness, float dampingRatio);
    JointTorques Feedforward(const ArmModel& model, const MeasuredJoints& measured, const JointReference& reference);

    class JointImpedance
        : public ArmController
    {
    public:
        JointImpedance(const DrivenArm& drivenArm, const JointImpedanceTuning& tuning);

        bool SetStiffnessShare(float share);
        float StiffnessShare() const;
        ArmModel::MassMatrix Damping(const JointPositions& positions) const;

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        void Size();

        const DrivenArm& drivenArm;
        const ArmModel& model;
        JointStiffnesses profileStiffness;
        float dampingRatio;
        float share{ 1.0f };
        JointStiffnesses stiffness{};
    };
}

#endif
