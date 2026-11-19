#include "tools/arm_host/ArmCommand.hpp"
#include <array>

namespace arm_host
{
    std::string_view CommandName(const ArmCommand& command)
    {
        static constexpr std::array<std::string_view, std::variant_size_v<ArmCommand>> names{ "check contracts", "select mode", "start", "stop",
            "clear faults", "emergency stop", "re-arm", "move", "abort", "inject fault", "silence joint", "bring up", "enable", "disable", "apply controller",
            "read controller", "apply motor", "read motor", "enable arm controller", "disable arm controller", "select arm controller", "align", "restart motor", "resume joint", "apply torque", "power off motor", "step" };

        return names[command.index()];
    }

    std::string_view MisconfigurationText(const arm::Misconfiguration& misconfiguration)
    {
        const bool differs = misconfiguration.reason == arm::MisconfigurationReason::differs;
        if (misconfiguration.part == arm::ConfigurationPart::motor)
            return differs ? "motor reads back different" : "motor could not be configured";
        if (misconfiguration.part == arm::ConfigurationPart::watchdog)
            return differs ? "drive watchdog reads back different" : "drive watchdog could not be configured";
        return differs ? "motor controller reads back different" : "motor controller could not be configured";
    }

    std::string_view StateName(arm::ArmState state)
    {
        static constexpr std::array<std::string_view, 13> names{ "unpowered", "checking contracts", "contracts incompatible", "waiting for calibration",
            "configuring motor controllers", "misconfigured", "ready", "enabling", "enabled", "moving", "disabling", "holding", "tripped" };

        return names[static_cast<std::size_t>(state)];
    }
}
