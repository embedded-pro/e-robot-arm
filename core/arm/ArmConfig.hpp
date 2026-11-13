#ifndef CORE_ARM_ARM_CONFIG_HPP
#define CORE_ARM_ARM_CONFIG_HPP

#include "core/arm_model/ArmGeometry.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "core/joint_configurator/interfaces/MotorSettings.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "e-foc-protocol/contract/FocMotorMessages.hpp"
#include "infra/timer/Timer.hpp"
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
        can::FocMotorMode enableMode{ can::FocMotorMode::position };
        JointControllers controllers{};
        JointMotors motors{};
        ArmGeometry geometry{ ReferenceGeometry() };
    };
}

#endif
