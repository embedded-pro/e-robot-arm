#ifndef TOOLS_SIMULATOR_VIEW_MODELS_CONTROL_RULES_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_CONTROL_RULES_HPP

#include "tools/arm_host/ArmSnapshot.hpp"
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
        target
    };

    struct ControlState
    {
        bool enabled;
        std::string reason;
    };

    ControlState Evaluate(Control control, const arm_host::ArmSnapshot& snapshot, std::string_view targetProblem = {});
}

#endif
