#include "core/arm/Arm.hpp"

namespace arm
{
    Arm::Arm(can::FocMotorCategoryClient& client, const ArmConfig& config)
        : jointBus(client, config.nodes, config.missedRequestsBeforeSilent)
        , safety(jointBus, config.positionLimits)
        , motion(safety, config.positionLimits, config.motionLimits, config.controlPeriod)
        , loop(safety, config.controlPeriod, [this]()
              {
                  motion.Step();
              })
    {}

    SafetySupervisor& Arm::Safety()
    {
        return safety;
    }

    JointMotion& Arm::Motion()
    {
        return motion;
    }

    ControlLoop& Arm::Loop()
    {
        return loop;
    }
}
