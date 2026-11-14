#include "core/arm_controllers/implementations/JointPositionPassThrough.hpp"

namespace arm
{
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
