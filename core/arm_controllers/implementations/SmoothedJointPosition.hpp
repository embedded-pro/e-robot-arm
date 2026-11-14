#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_SMOOTHED_JOINT_POSITION_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_SMOOTHED_JOINT_POSITION_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "infra/timer/Timer.hpp"
#include "numerical/filters/passive/ExponentialMovingAverage.hpp"
#include <array>

namespace arm
{
    class SmoothedJointPosition
        : public ArmController
    {
    public:
        SmoothedJointPosition(infra::Duration period, infra::Duration timeConstant);

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointPositions& reference) override;

    private:
        using Filter = filters::passive::ExponentialMovingAverage<float>;

        std::array<Filter, jointCount> filters;
    };
}

#endif
