#include "tools/arm_host/ArmCommand.hpp"
#include <array>

namespace arm_host
{
    std::string_view CommandName(const ArmCommand& command)
    {
        static constexpr std::array<std::string_view, std::variant_size_v<ArmCommand>> names{ "check contracts", "select mode", "start", "stop",
            "clear faults", "emergency stop", "re-arm", "move", "abort", "inject fault", "silence joint", "bring up", "enable", "disable", "apply controller",
            "read controller" };

        return names[command.index()];
    }

    std::string_view StateName(arm::ArmState state)
    {
        static constexpr std::array<std::string_view, 10> names{ "unpowered", "checking contracts", "contracts incompatible", "waiting for calibration", "ready",
            "enabling", "enabled", "moving", "disabling", "tripped" };

        return names[static_cast<std::size_t>(state)];
    }
}
