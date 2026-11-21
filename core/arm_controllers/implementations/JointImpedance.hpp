#pragma once

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/ArmModel.hpp"
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

    ArmModel::MassMatrix HomeInertia(const ArmModel& model, const JointInertias& rotors);
    JointStiffnesses ImpedanceStiffness(const ArmModel::MassMatrix& inertia, const JointImpedanceTuning& tuning);
    ArmModel::MassMatrix CoupledDamping(const ArmModel::MassMatrix& inertia, const JointStiffnesses& stiffness, float dampingRatio);
    JointTorques Feedforward(const ArmModel& model, const MeasuredJoints& measured, const JointReference& reference);

    class JointImpedance
        : public ArmController
    {
    public:
        JointImpedance(const ArmModel& model, const JointInertias& rotors, const JointImpedanceTuning& tuning);

        bool SetStiffnessShare(float share);
        float StiffnessShare() const;
        ArmModel::MassMatrix Damping(const JointPositions& positions) const;

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        void Size();

        const ArmModel& model;
        JointInertias rotors;
        JointStiffnesses profileStiffness;
        float dampingRatio;
        float share{ 1.0f };
        JointStiffnesses stiffness{};
    };
}
