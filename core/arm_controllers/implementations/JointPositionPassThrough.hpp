#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_JOINT_POSITION_PASS_THROUGH_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_JOINT_POSITION_PASS_THROUGH_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"

namespace arm
{
    class JointPositionPassThrough
        : public ArmController
    {
    public:
        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointPositions& reference) override;
    };
}

#endif
