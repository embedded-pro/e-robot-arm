#ifndef TOOLS_SIMULATOR_VIEW_MODELS_CONTROL_RULES_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_CONTROL_RULES_HPP

#include "tools/arm_host/ArmSnapshot.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace simulator
{
    enum class Control : uint8_t
    {
        emergencyStop,
        checkContracts,
        positionMode,
        start,
        stop,
        clearFaults,
        rearm,
        move,
        abort,
        target,
        bringUp,
        enable,
        disable
    };

    inline constexpr std::size_t controlCount{ 13 };

    struct ControlState
    {
        bool enabled;
        std::string reason;
    };

    ControlState Evaluate(Control control, const arm_host::ArmSnapshot& snapshot, std::string_view targetProblem = {});
}

#endif
