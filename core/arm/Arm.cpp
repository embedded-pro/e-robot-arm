#include "core/arm/Arm.hpp"
#include <algorithm>
#include <limits>

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

    foc::NewtonMeter TorqueLimit(const ArmConfig& config, std::size_t joint)
    {
        if (config.torqueLimits[joint].has_value())
            return *config.torqueLimits[joint];
        if (!config.motors[joint].has_value())
            return foc::NewtonMeter{ std::numeric_limits<float>::infinity() };
        const auto& motor = config.motors[joint]->settings;
        const auto torqueConstant = 1.5f * motor[MotorParameter::polePairs] * motor[MotorParameter::fluxLinkage];
        return foc::NewtonMeter{ motor[MotorParameter::peakCurrent] * torqueConstant * config.transmissions[joint].ratio };
    }

    MotionSupervision Supervision(const ArmConfig& config)
    {
        MotionSupervision supervision{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto& motion = config.motionLimits[joint];
            supervision.reference[joint] = { motion.maxSpeed, motion.maxAcceleration };
            supervision.speed[joint] = config.speedTrips[joint].value_or(foc::RadiansPerSecond{ defaultSpeedTripShare * motion.maxSpeed.Value() });
            supervision.torque[joint] = TorqueLimit(config, joint);
        }
        return supervision;
    }

    PdGainMatrices GravityPdGains(const ArmModel& model, const ArmConfig& config)
    {
        return PlacedGains(model, ReflectedRotors(config), config.gravityPd);
    }

    Arm::Arm(can::FocMotorCategoryClient& client, const ArmConfig& config, HardwareStop* hardwareStop)
        : jointBus(client, config.nodes, config.transmissions, config.missedRequestsBeforeSilent, config.controlPeriod * config.commandTimeoutPeriods)
        , safety(jointBus, config.positionLimits, config.tracking, config.collision, config.controlPeriod, Supervision(config), SupervisionInputs{ &workspace, hardwareStop })
        , motion(safety, config.positionLimits, config.motionLimits, config.controlPeriod, &workspace, config.jerkLimits, ToolLineSetup{ &model, config.toolLimits })
        , configurator(safety, lifecycle, SilenceWindow(config))
        , lifecycle(safety, motion, configurator, config.controllers, config.motors, DriveWatchdogSeconds(config))
        , model(config.geometry, config.mass, config.positionLimits)
        , workspace(model, config.geometry, config.workspace)
        , gravityPd(model, GravityPdGains(model, config))
        , computedTorque(model, ReflectedRotors(config), config.computedTorque)
        , gravityPid(model, ReflectedRotors(config), config.gravityPid, config.controlPeriod)
        , jointImpedance(model, ReflectedRotors(config), config.jointImpedance)
        , cartesianImpedance(model, ReflectedRotors(config), config.cartesianImpedance)
        , operationalSpace(model, ReflectedRotors(config), config.operationalSpace, config.toolLimits.manipulabilityMin)
        , slotineLi(model, ReflectedRotors(config), config.slotineLi, config.controlPeriod)
        , experimental(config.experimental)
        , collision(model, ReflectedRotors(config), config.collisionObserverGain)
        , period(config.controlPeriod)
        , torqueLimits(Supervision(config).torque)
        , controllers(safety, motion, config.controlPeriod)
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
        controllers.Enable(controllers.Register("joint-impedance", jointImpedance));
        controllers.Enable(controllers.Register("cartesian-impedance", cartesianImpedance));
        controllers.Enable(controllers.Register("operational-space", operationalSpace));
        if (experimental)
            controllers.Enable(controllers.Register("slotine-li", slotineLi));
        controllers.Select(gravityPdId);
        motion.SelectProfile(config.motionProfile);
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

    bool Arm::SetImpedanceShare(float share)
    {
        if (!jointImpedance.SetStiffnessShare(share))
            return false;
        cartesianImpedance.SetStiffnessShare(share);
        return true;
    }

    float Arm::ImpedanceShare() const
    {
        return jointImpedance.StiffnessShare();
    }

    bool Arm::ResetAdaptation()
    {
        if (!experimental)
            return false;
        slotineLi.ResetEstimate();
        return true;
    }

    const std::optional<JointTorques>& Arm::EstimatedExternalTorques() const
    {
        return externalTorques;
    }

    const WorkspaceCheck& Arm::Workspace() const
    {
        return workspace;
    }

    ControlLoop& Arm::Loop()
    {
        return loop;
    }

    void Arm::Step()
    {
        auto applied = controllers.LastSetpoints();
        if (applied.has_value())
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                applied->torques[joint] = foc::NewtonMeter{ std::clamp(applied->torques[joint].Value(), -torqueLimits[joint].Value(), torqueLimits[joint].Value()) };
        const auto reference = motion.Step();
        if (!controllers.Step(reference))
            motion.Abort();
        const auto tracked = controllers.TrackedReference();
        externalTorques = ExternalTorques(applied, tracked.has_value());
        safety.Supervise(tracked, externalTorques);
        lifecycle.Step();
    }

    std::optional<JointTorques> Arm::ExternalTorques(const std::optional<JointSetpoints>& applied, bool streaming)
    {
        if (!streaming || !applied.has_value())
        {
            observing = false;
            return std::nullopt;
        }

        JointPositions positions{};
        JointSpeeds speeds{};
        std::array<uint32_t, jointCount> readings{};
        infra::TimePoint request{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            positions[joint] = safety.Reading(joint).position;
            speeds[joint] = safety.Reading(joint).speed;
            readings[joint] = safety.Reading(joint).period;
            request = std::max(request, safety.Reading(joint).requested);
        }

        if (!observing)
        {
            collision.Reset(positions, speeds);
            previousApplied = applied->torques;
            actingSum = {};
            actingPeriods = 0;
            observedReadings = readings;
            observedRequest = request;
            observing = true;
            return std::nullopt;
        }

        for (std::size_t joint = 0; joint != jointCount; ++joint)
            actingSum[joint] = foc::NewtonMeter{ actingSum[joint].Value() + 0.5f * (applied->torques[joint].Value() + previousApplied[joint].Value()) };
        previousApplied = applied->torques;
        ++actingPeriods;

        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (readings[joint] == observedReadings[joint])
                return externalTorques;

        JointTorques acting{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            acting[joint] = foc::NewtonMeter{ actingSum[joint].Value() / static_cast<float>(actingPeriods) };
        const auto elapsed = request > observedRequest ? request - observedRequest : period * actingPeriods;
        actingSum = {};
        actingPeriods = 0;
        observedReadings = readings;
        observedRequest = request;
        return collision.Update(positions, speeds, acting, elapsed);
    }
}
