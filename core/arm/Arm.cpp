#include "core/arm/Arm.hpp"

namespace arm
{
    JointInertias ReflectedRotors(const ArmConfig& config)
    {
        JointInertias rotors{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto& motor = config.motors[joint].has_value() ? config.motors[joint]->settings : motorPresets[0].settings;
            const auto ratio = config.transmissions[joint].ratio;
            rotors[joint] = motor[MotorParameter::inertia] * ratio * ratio;
        }
        return rotors;
    }

    PdGainMatrices GravityPdGains(const ArmModel& model, const ArmConfig& config)
    {
        return PlacedGains(model, ReflectedRotors(config), config.gravityPd);
    }

    Arm::Arm(can::FocMotorCategoryClient& client, const ArmConfig& config)
        : jointBus(client, config.nodes, config.transmissions, config.missedRequestsBeforeSilent, config.controlPeriod * config.commandTimeoutPeriods)
        , safety(jointBus, config.positionLimits, config.tracking, config.controlPeriod)
        , motion(safety, config.positionLimits, config.motionLimits, config.controlPeriod)
        , configurator(safety, lifecycle, SilenceWindow(config))
        , lifecycle(safety, motion, configurator, config.controllers, config.motors)
        , model(config.geometry, config.mass, config.positionLimits)
        , gravityPd(model, GravityPdGains(model, config))
        , computedTorque(model, ReflectedRotors(config), config.computedTorque)
        , gravityPid(model, ReflectedRotors(config), config.gravityPid, config.controlPeriod)
        , controllers(safety, motion)
        , engagement(lifecycle, controllers)
        , loop(safety, config.controlPeriod, [this]()
              {
                  Step();
              })
    {
        const auto gravityPdId = controllers.Register("gravity-pd", gravityPd);
        controllers.Enable(gravityPdId);
        controllers.Enable(controllers.Register("computed-torque", computedTorque));
        controllers.Enable(controllers.Register("gravity-pid", gravityPid));
        controllers.Select(gravityPdId);
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
        safety.Supervise(controllers.Reference());
        lifecycle.Step();
    }
}
