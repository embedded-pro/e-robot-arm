#ifndef TOOLS_ARM_HOST_ARM_EVENT_HPP
#define TOOLS_ARM_HOST_ARM_EVENT_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "core/joint_configurator/interfaces/MotorSettings.hpp"
#include "core/lifecycle/interfaces/ArmLifecycle.hpp"
#include "infra/timer/Timer.hpp"
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

namespace arm_host
{
    enum class ArmEventKind : uint8_t
    {
        accepted,
        refused,
        contractsResolved,
        jointFault,
        jointSilent,
        tripped,
        moveDone,
        moveAborted,
        targetReady,
        targetFailed,
        stateChanged,
        controllerApplied,
        controllerRead,
        controllerFailed,
        motorApplied,
        motorRead,
        profileLoaded,
        armControllerChanged
    };

    inline constexpr std::size_t noJoint{ std::numeric_limits<std::size_t>::max() };

    struct ArmEvent
    {
        ArmEventKind kind;
        infra::TimePoint time{};
        uint32_t command{ 0 };
        std::string_view commandName{};
        std::string_view reason{};
        std::size_t joint{ noJoint };
        can::FocFaultCode fault{ can::FocFaultCode::none };
        can::ContractCompatibility contracts{ can::ContractCompatibility::unknown };
        arm::ArmState state{ arm::ArmState::unpowered };
        arm::ControllerReading controller{};
        arm::MotorSettings motor{};
        std::string_view armController{};
    };
}

#endif
