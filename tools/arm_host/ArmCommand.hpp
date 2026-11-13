#ifndef TOOLS_ARM_HOST_ARM_COMMAND_HPP
#define TOOLS_ARM_HOST_ARM_COMMAND_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "core/lifecycle/interfaces/ArmLifecycle.hpp"
#include <cstddef>
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

    struct EmergencyStop
    {};

    struct Rearm
    {};

    struct MoveTo
    {
        arm::JointPositions target;
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

    using ArmCommand = std::variant<CheckContracts, SelectMode, StartJoints, StopJoints, ClearFaults, EmergencyStop, Rearm, MoveTo, AbortMove, InjectFault, SilenceJoint,
        BringUp, Enable, Disable, ApplyController, ReadController>;

    std::string_view CommandName(const ArmCommand& command);
    std::string_view StateName(arm::ArmState state);
}

#endif
