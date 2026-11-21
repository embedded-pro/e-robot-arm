#pragma once

#include "core/arm_profile/ArmProfile.hpp"
#include <filesystem>
#include <string>
#include <variant>

namespace arm_host
{
    std::variant<arm::ArmProfile, std::string> ReadProfileFile(const std::filesystem::path& path);
    std::string WriteProfileFile(const std::filesystem::path& path, const arm::ArmProfile& profile);
}
