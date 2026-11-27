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

    Arm::Arm(can::FocMotorCategoryClient& client, const ArmConfig& config, HardwareStop* hardwareStop)
        : jointBus(client, config.nodes, config.transmissions, config.missedRequestsBeforeSilent, config.controlPeriod * config.commandTimeoutPeriods)
        , safety(jointBus, config.positionLimits, config.tracking, config.collision, config.controlPeriod, Supervision(config), SupervisionInputs{ &workspace, hardwareStop })
        , motion(safety, config.positionLimits, config.motionLimits, config.controlPeriod, &workspace, config.jerkLimits, ToolLineSetup{ &model, config.toolLimits })
        , configurator(safety, lifecycle, SilenceWindow(config))
        // Only the address is stored; the lifecycle asks the permit after construction
        , lifecycle(safety, motion, configurator, config.controllers, config.motors, DriveWatchdogSeconds(config), &engagement)
        , model(config.geometry, config.mass, config.positionLimits)
        , drivenArm(model, ReflectedRotors(config))
        , workspace(model, config.geometry, config.workspace)
        , gravityPd(drivenArm, config.gravityPd)
        , computedTorque(drivenArm, config.computedTorque)
        , gravityPid(drivenArm, config.gravityPid, config.controlPeriod)
        , jointImpedance(drivenArm, config.jointImpedance)
        , cartesianImpedance(drivenArm, config.cartesianImpedance)
        , operationalSpace(drivenArm, config.operationalSpace, config.toolLimits.manipulabilityMin)
        , slotineLi(drivenArm, config.slotineLi, config.controlPeriod)
        , freeController(drivenArm, config.positionLimits, PassiveLaw::free)
        , gravityFloat(drivenArm, config.positionLimits, PassiveLaw::gravityFloat)
        , experimental(config.experimental)
        , collision(drivenArm, config.collisionObserverGain)
        , period(config.controlPeriod)
        , torqueLimits(Supervision(config).torque)
        , controllers(safety, motion, config.controlPeriod)
        , engagement(lifecycle, controllers)
        , loop(safety, config.controlPeriod, [this]()
              {
                  Step();
              })
    {
        const auto gravityPdId = controllers.Register(ArmControllerName(ArmControllerKind::gravityPd), gravityPd);
        controllers.Enable(gravityPdId);
        controllers.Enable(controllers.Register(ArmControllerName(ArmControllerKind::computedTorque), computedTorque));
        controllers.Enable(controllers.Register(ArmControllerName(ArmControllerKind::gravityPid), gravityPid));
        controllers.Enable(controllers.Register(ArmControllerName(ArmControllerKind::jointImpedance), jointImpedance));
        controllers.Enable(controllers.Register(ArmControllerName(ArmControllerKind::cartesianImpedance), cartesianImpedance));
        controllers.Enable(controllers.Register(ArmControllerName(ArmControllerKind::operationalSpace), operationalSpace));
        if (experimental)
            controllers.Enable(controllers.Register(ArmControllerName(ArmControllerKind::slotineLi), slotineLi));
        if (config.simulation)
        {
            controllers.Enable(controllers.Register(ArmControllerName(ArmControllerKind::free), freeController, true));
            controllers.Enable(controllers.Register(ArmControllerName(ArmControllerKind::gravityFloat), gravityFloat, true));
        }
        controllers.Select(controllers.Find(ArmControllerName(config.activeController)).value_or(gravityPdId));
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

    RetuneResult Arm::Retune(ArmControllerKind kind, const ArmControllerTuning& tuning)
    {
        const auto id = controllers.Find(ArmControllerName(kind));
        if (!id.has_value() || !TuningFits(kind, tuning))
            return RetuneResult::notTunable;
        if (CheckTuning(tuning).has_value())
            return RetuneResult::outOfRange;
        if (motion.Moving())
            return RetuneResult::moving;

        Apply(kind, tuning);
        return RetuneResult::applied;
    }

    void Arm::Apply(ArmControllerKind kind, const ArmControllerTuning& tuning)
    {
        switch (kind)
        {
            case ArmControllerKind::gravityPd:
                gravityPd.SetTuning(std::get<BandwidthTuning>(tuning));
                break;
            case ArmControllerKind::computedTorque:
                computedTorque.SetTuning(std::get<BandwidthTuning>(tuning));
                break;
            case ArmControllerKind::gravityPid:
                gravityPid.SetTuning(std::get<BandwidthTuning>(tuning));
                break;
            case ArmControllerKind::jointImpedance:
                jointImpedance.SetTuning(std::get<JointImpedanceTuning>(tuning));
                break;
            case ArmControllerKind::cartesianImpedance:
                cartesianImpedance.SetTuning(std::get<CartesianImpedanceTuning>(tuning));
                break;
            case ArmControllerKind::operationalSpace:
                operationalSpace.SetTuning(std::get<OperationalSpaceTuning>(tuning));
                break;
            case ArmControllerKind::slotineLi:
                slotineLi.SetTuning(std::get<SlotineLiTuning>(tuning));
                break;
            default:
                break;
        }
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
            std::ranges::transform(applied->torques, torqueLimits, applied->torques.begin(), [](foc::NewtonMeter torque, foc::NewtonMeter limit)
                {
                    return foc::NewtonMeter{ std::clamp(torque.Value(), -limit.Value(), limit.Value()) };
                });
        const auto reference = motion.Step();
        if (!controllers.Step(reference))
            motion.Abort();
        if (controllers.StreamingPassive())
        {
            externalTorques = ExternalTorques(applied, false);
            safety.SupervisePassive();
        }
        else
        {
            const auto tracked = controllers.TrackedReference();
            externalTorques = ExternalTorques(applied, tracked.has_value());
            safety.Supervise(tracked, externalTorques);
        }
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

        actingSum += (ToVector(applied->torques) + ToVector(previousApplied)) * 0.5f;
        previousApplied = applied->torques;
        ++actingPeriods;

        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (readings[joint] == observedReadings[joint])
                return externalTorques;

        const auto acting = FromVector<foc::NewtonMeter>(actingSum * (1.0f / static_cast<float>(actingPeriods)));
        const auto elapsed = request > observedRequest ? request - observedRequest : period * actingPeriods;
        actingSum = {};
        actingPeriods = 0;
        observedReadings = readings;
        observedRequest = request;
        return collision.Update(positions, speeds, acting, elapsed);
    }
}
