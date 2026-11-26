#include "core/arm_controllers/implementations/PassiveController.hpp"
#include <algorithm>

namespace arm
{
    PassiveController::PassiveController(const DrivenArm& drivenArm, const JointLimits& limits, PassiveLaw law, const JointLimitWall& wall)
        : drivenArm(drivenArm)
        , limits(limits)
        , law(law)
        , wall(wall)
    {}

    bool PassiveController::BumplessTransfer() const
    {
        return false;
    }

    void PassiveController::Reset(const MeasuredJoints&, const JointReference&)
    {}

    JointSetpoints PassiveController::Step(const MeasuredJoints& measured, const JointReference&)
    {
        JointSetpoints setpoints{ measured.positions, {} };
        if (law == PassiveLaw::gravityFloat)
            setpoints.torques = drivenArm.Model().GravityTorque(measured.positions);

        const auto inertia = drivenArm.JointMassMatrix(measured.positions);
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            setpoints.torques[joint] = foc::NewtonMeter{ setpoints.torques[joint].Value() + WallTorque(joint, measured, inertia.at(joint, joint)) };
        return setpoints;
    }

    float PassiveController::WallTorque(std::size_t joint, const MeasuredJoints& measured, float inertia) const
    {
        const auto lower = limits[joint].lower.Value();
        const auto upper = limits[joint].upper.Value();
        const auto band = std::min(wall.band.Value(), 0.5f * (upper - lower));
        const auto position = measured.positions[joint].Value();
        const auto depth = std::max(position - (upper - band), 0.0f) - std::max(lower + band - position, 0.0f);
        if (position > lower + band && position < upper - band)
            return 0.0f;

        const auto bandwidth = wall.bandwidth;
        return -bandwidth * bandwidth * inertia * depth - 2.0f * bandwidth * inertia * measured.speeds[joint].Value();
    }
}
