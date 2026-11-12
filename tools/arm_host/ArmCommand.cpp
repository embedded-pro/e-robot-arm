#include "tools/arm_host/ArmCommand.hpp"
#include <array>

namespace arm_host
{
    std::string_view CommandName(const ArmCommand& command)
    {
        static constexpr std::array<std::string_view, std::variant_size_v<ArmCommand>> names{ "check contracts", "select mode", "start", "stop",
            "clear faults", "emergency stop", "re-arm", "move", "abort", "inject fault", "silence joint" };

        return names[command.index()];
    }
}
