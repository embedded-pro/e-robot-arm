#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_OPERATIONAL_SPACE_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_OPERATIONAL_SPACE_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/ArmTaskJacobian.hpp"
#include "core/arm_model/DrivenArm.hpp"
#include "robotics/controllers/manipulator/OperationalSpaceControl.hpp"

namespace arm
{
    inline constexpr float fullJointSpaceBelow{ 0.75f };

    struct OperationalSpaceTuning
    {
        float bandwidth{ 30.0f };
        float dampingRatio{ 0.7f };
        float singularityDamping{ 0.01f };

        bool operator==(const OperationalSpaceTuning& other) const = default;
    };

    class OperationalSpace
        : public ArmController
    {
    public:
        OperationalSpace(const DrivenArm& drivenArm, const OperationalSpaceTuning& tuning, float manipulabilityMin);

        float JointSpaceShare(const JointPositions& positions) const;

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        using Law = controllers::OperationalSpaceControl<float, jointCount, 6>;

        ArmModel::JointVector JointSpaceTorques(const ArmModel::JointVector& angles, const ArmModel::JointVector& speeds, const JointReference& reference) const;

        const ArmModel& model;
        ArmTaskJacobian provider;
        const DrivenArm::Dynamics& dynamics;
        Law law;
        float stiffness;
        float damping;
        float manipulabilityMin;
    };
}

#endif
