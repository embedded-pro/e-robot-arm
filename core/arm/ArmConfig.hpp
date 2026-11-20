#ifndef CORE_ARM_ARM_CONFIG_HPP
#define CORE_ARM_ARM_CONFIG_HPP

#include "core/arm_controllers/implementations/CartesianImpedance.hpp"
#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"
#include "core/arm_controllers/implementations/JointImpedance.hpp"
#include "core/arm_controllers/implementations/OperationalSpace.hpp"
#include "core/arm_model/ArmGeometry.hpp"
#include "core/arm_model/ArmMass.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "core/joint_configurator/interfaces/MotorSettings.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "core/safety/interfaces/SafetySupervisor.hpp"
#include "core/workspace/WorkspaceConfig.hpp"
#include "e-foc-protocol/contract/FocMotorMessages.hpp"
#include "infra/timer/Timer.hpp"
#include "infra/util/BoundedString.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace arm
{
    inline constexpr std::size_t maxTeachPoints{ 16 };
    inline constexpr std::size_t teachPointNameSize{ 16 };

    struct TeachPoint
    {
        infra::BoundedString::WithStorage<teachPointNameSize> name;
        JointPositions joints{};
    };

    struct ArmConfig
    {
        JointNodeIds nodes;
        uint8_t missedRequestsBeforeSilent;
        JointLimits positionLimits;
        JointMotionLimits motionLimits;
        infra::Duration controlPeriod;
        uint8_t commandTimeoutPeriods{ 3 };
        uint8_t driveWatchdogPeriods{ 10 };
        JointTransmissions transmissions{};
        JointControllers controllers{};
        JointMotors motors{};
        ArmGeometry geometry{ ReferenceGeometry() };
        ArmMass mass{ ReferenceMass() };
        BandwidthTuning gravityPd{};
        BandwidthTuning computedTorque{};
        BandwidthTuning gravityPid{ 20.0f, 0.7f };
        JointImpedanceTuning jointImpedance{};
        CartesianImpedanceTuning cartesianImpedance{};
        OperationalSpaceTuning operationalSpace{};
        TrackingLimits tracking{};
        CollisionLimits collision{ DefaultCollisionLimits() };
        std::array<std::optional<foc::RadiansPerSecond>, jointCount> speedTrips{};
        std::array<std::optional<foc::NewtonMeter>, jointCount> torqueLimits{};
        WorkspaceConfig workspace{};
        float collisionObserverGain{ 25.0f };
        JointJerkLimits jerkLimits{};
        MotionProfile motionProfile{ MotionProfile::trapezoid };
        ToolLimits toolLimits{};
        std::array<TeachPoint, maxTeachPoints> teachPoints{};
        uint8_t teachPointCount{ 0 };
    };

    inline constexpr float defaultSpeedTripShare{ 4.0f };

    inline float DriveWatchdogSeconds(const ArmConfig& config)
    {
        return std::chrono::duration<float>{ config.controlPeriod * config.driveWatchdogPeriods }.count();
    }

    inline infra::Duration SilenceWindow(const ArmConfig& config)
    {
        return config.controlPeriod * config.missedRequestsBeforeSilent;
    }
}

#endif
