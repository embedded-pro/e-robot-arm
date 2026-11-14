#include "core/arm/Arm.hpp"

namespace arm
{
    Arm::Arm(can::FocMotorCategoryClient& client, const ArmConfig& config)
        : jointBus(client, config.nodes, config.transmissions, config.missedRequestsBeforeSilent, config.controlPeriod * config.commandTimeoutPeriods)
        , safety(jointBus, config.positionLimits)
        , motion(safety, config.positionLimits, config.motionLimits, config.controlPeriod)
        , configurator(safety, lifecycle)
        , lifecycle(safety, motion, configurator, config.enableMode, config.controllers, config.motors)
        , smoothed(config.controlPeriod, config.smoothingTimeConstant)
        , controllers(safety, motion)
        , engagement(lifecycle, controllers)
        , loop(safety, config.controlPeriod, [this]()
              {
                  Step();
              })
    {
        const auto passThroughId = controllers.Register("pass-through", passThrough);
        controllers.Enable(passThroughId);
        controllers.Enable(controllers.Register("smoothed", smoothed));
        controllers.Select(passThroughId);
    }

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

    ArmControllers& Arm::Controllers()
    {
        return controllers;
    }

    const ArmControllers& Arm::Controllers() const
    {
        return controllers;
    }

    ControlLoop& Arm::Loop()
    {
        return loop;
    }

    void Arm::Step()
    {
        const auto reference = motion.Step();
        if (!controllers.Step(reference))
            motion.Abort();
        lifecycle.Step();
    }
}
