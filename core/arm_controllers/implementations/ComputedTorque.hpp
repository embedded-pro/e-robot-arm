#pragma once

#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"
#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/DrivenArm.hpp"
#include "robotics/controllers/manipulator/ComputedTorqueControl.hpp"

namespace arm
{
    class ComputedTorque
        : public ArmController
    {
    public:
        ComputedTorque(const DrivenArm& drivenArm, const BandwidthTuning& tuning);

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        using Law = controllers::ComputedTorqueControl<float, jointCount>;

        const ArmModel& model;
        Law law;
    };
}
