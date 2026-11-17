#ifndef CORE_ARM_ARM_CONFIG_HPP
#define CORE_ARM_ARM_CONFIG_HPP

#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"
#include "core/arm_model/ArmGeometry.hpp"
#include "core/arm_model/ArmMass.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "core/joint_configurator/interfaces/MotorSettings.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "core/safety/interfaces/SafetySupervisor.hpp"
#include "e-foc-protocol/contract/FocMotorMessages.hpp"
#include "infra/timer/Timer.hpp"
#include <chrono>
#include <cstdint>

namespace arm
{
    struct ArmConfig
    {
        JointNodeIds nodes;
        uint8_t missedRequestsBeforeSilent;
        JointLimits positionLimits;
        JointMotionLimits motionLimits;
        infra::Duration controlPeriod;
        uint8_t commandTimeoutPeriods{ 3 };
        JointTransmissions transmissions{};
        JointControllers controllers{};
        JointMotors motors{};
        ArmGeometry geometry{ ReferenceGeometry() };
        ArmMass mass{ ReferenceMass() };
        BandwidthTuning gravityPd{};
        BandwidthTuning computedTorque{};
        BandwidthTuning gravityPid{ 20.0f, 0.7f };
        TrackingLimits tracking{};
        CollisionLimits collision{ DefaultCollisionLimits() };
        float collisionObserverGain{ 50.0f };
    };

    inline infra::Duration SilenceWindow(const ArmConfig& config)
    {
        return config.controlPeriod * config.missedRequestsBeforeSilent;
    }
}

#endif
