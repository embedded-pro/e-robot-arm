#ifndef TOOLS_ARM_HOST_ARM_COMMAND_HPP
#define TOOLS_ARM_HOST_ARM_COMMAND_HPP

#include "core/arm_controllers/interfaces/ArmControllers.hpp"
#include "core/arm_model/ArmGeometry.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "core/joint_configurator/interfaces/MotorSettings.hpp"
#include "core/lifecycle/interfaces/ArmLifecycle.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include <cstddef>
#include <string>
#include <string_view>
#include <variant>

namespace arm_host
{
    struct CheckContracts
    {};

    struct SelectMode
    {
        can::FocMotorMode mode;
    };

    struct StartJoints
    {};

    struct StopJoints
    {};

    struct ClearFaults
    {};

    struct AlignJoints
    {};

    struct EmergencyStop
    {};

    struct Rearm
    {};

    struct MoveTo
    {
        arm::JointPositions target;
    };

    struct StepTo
    {
        arm::JointPositions target;
    };

    struct MoveThrough
    {
        arm::JointWaypoints::WithMaxSize<arm::maxWaypoints> waypoints;
    };

    struct MoveLine
    {
        arm::Placement target;
    };

    struct JogJoint
    {
        std::size_t joint;
        foc::RadiansPerSecond speed;
    };

    struct JogTool
    {
        arm::ToolJog jog;
    };

    using AnyJog = std::variant<arm::JointJog, arm::ToolJog>;

    constexpr infra::Duration maxHeldJog{ std::chrono::seconds{ 10 } };
    constexpr uint32_t heldJogRefreshPeriods{ 5 };

    struct HoldJog
    {
        AnyJog jog;
        infra::Duration duration;
    };

    struct Teach
    {
        std::string name;
    };

    struct GoTo
    {
        std::string name;
    };

    struct AbortMove
    {};

    struct InjectFault
    {
        std::size_t joint;
        can::FocFaultCode fault;
    };

    struct SilenceJoint
    {
        std::size_t joint;
    };

    struct RestartMotor
    {
        std::size_t joint;
    };

    struct ResumeJoint
    {
        std::size_t joint;
    };

    struct CollideJoint
    {
        std::size_t joint;
        foc::NewtonMeter torque;
    };

    struct PowerOffMotor
    {
        std::size_t joint;
    };

    struct BringUp
    {};

    struct Enable
    {};

    struct Disable
    {};

    struct ApplyController
    {
        std::size_t joint;
        arm::ControllerSettings settings;
    };

    struct ReadController
    {
        std::size_t joint;
    };

    struct ApplyMotor
    {
        std::size_t joint;
        arm::MotorSettings settings;
    };

    struct ReadMotor
    {
        std::size_t joint;
    };

    struct EnableArmController
    {
        arm::ArmControllerId id;
    };

    struct DisableArmController
    {
        arm::ArmControllerId id;
    };

    struct SelectArmController
    {
        arm::ArmControllerId id;
    };

    struct SetImpedanceShare
    {
        float share;
    };

    struct SelectMotionProfile
    {
        arm::MotionProfile profile;
    };

    using ArmCommand = std::variant<CheckContracts, SelectMode, StartJoints, StopJoints, ClearFaults, EmergencyStop, Rearm, MoveTo, AbortMove, InjectFault, SilenceJoint,
        BringUp, Enable, Disable, ApplyController, ReadController, ApplyMotor, ReadMotor, EnableArmController, DisableArmController, SelectArmController, AlignJoints, RestartMotor, ResumeJoint, CollideJoint, PowerOffMotor, StepTo, SelectMotionProfile, MoveThrough, MoveLine, JogJoint, JogTool, HoldJog, Teach, GoTo, SetImpedanceShare>;

    std::string_view CommandName(const ArmCommand& command);
    std::string_view StateName(arm::ArmState state);
    std::string_view MisconfigurationText(const arm::Misconfiguration& misconfiguration);
}

#endif
