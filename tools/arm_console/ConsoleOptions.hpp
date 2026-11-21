#pragma once

#include "core/joint_bus/interfaces/JointBus.hpp"
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace arm_console
{
    struct ConsoleOptions
    {
        std::string bus{ "virtual:arm" };
        uint32_t bitrate{ 1000000 };
        arm::JointNodeIds nodes{ 1, 2, 3, 4, 5, 6 };
        std::string profile{};
        std::string virtualArm{};
        std::string engine{};
    };

    std::optional<ConsoleOptions> ParseOptions(const std::vector<std::string>& arguments);
    std::string OptionsUsage();
}
