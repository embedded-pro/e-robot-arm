#include "core/arm/Arm.hpp"

namespace arm
{
    Arm::Arm(can::FocMotorCategoryClient& client, const ArmConfig& config)
        : jointBus(client, config.nodes, config.transmissions, config.missedRequestsBeforeSilent, config.controlPeriod * config.commandTimeoutPeriods)
        , safety(jointBus, config.positionLimits)
        , motion(safety, config.positionLimits, config.motionLimits, config.controlPeriod)
        , configurator(safety, lifecycle)
        , lifecycle(safety, motion, configurator, config.enableMode, config.controllers)
        , loop(safety, config.controlPeriod, [this]()
              {
                  motion.Step();
                  lifecycle.Step();
              })
    {}

    SafetySupervisor& Arm::Safety()
    {
        return safety;
    }

    const SafetySupervisor& Arm::Safety() const
    {
        return safety;
    }

    JointMotion& Arm::Motion()
    {
        return motion;
    }

    const JointMotion& Arm::Motion() const
    {
        return motion;
    }

    JointConfigurator& Arm::Configurator()
    {
        return configurator;
    }

    const JointConfigurator& Arm::Configurator() const
    {
        return configurator;
    }

    ArmLifecycle& Arm::Lifecycle()
    {
        return lifecycle;
    }

    const ArmLifecycle& Arm::Lifecycle() const
    {
        return lifecycle;
    }

    ControlLoop& Arm::Loop()
    {
        return loop;
    }
}
