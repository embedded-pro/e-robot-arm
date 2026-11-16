#include "core/arm_controllers/implementations/JointPositionPassThrough.hpp"

namespace arm
{
    can::FocMotorMode JointPositionPassThrough::Mode() const
    {
        return can::FocMotorMode::position;
    }

    bool JointPositionPassThrough::BumplessTransfer() const
    {
        return false;
    }

    void JointPositionPassThrough::Reset(const MeasuredJoints&)
    {}

    JointSetpoints JointPositionPassThrough::Step(const MeasuredJoints&, const JointPositions& reference)
    {
        return JointSetpoints{ reference };
    }
}
