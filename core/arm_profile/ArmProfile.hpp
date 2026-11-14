#ifndef CORE_ARM_PROFILE_ARM_PROFILE_HPP
#define CORE_ARM_PROFILE_ARM_PROFILE_HPP

#include "core/arm/ArmConfig.hpp"
#include "infra/stream/OutputStream.hpp"
#include "infra/util/BoundedString.hpp"
#include <array>
#include <cstddef>
#include <optional>

namespace arm
{
    inline constexpr std::size_t profileNameSize{ 48 };
    inline constexpr std::size_t profileProblemSize{ 128 };
    inline constexpr int32_t profileVersion{ 1 };
    inline constexpr std::size_t meshPathSize{ 96 };
    inline constexpr std::size_t meshCount{ jointCount + 1 };
    inline constexpr std::array<const char*, meshCount> meshNames{ "base", "link1", "link2", "link3", "link4", "link5", "link6" };

    struct ArmProfile
    {
        infra::BoundedString::WithStorage<profileNameSize> name;
        ArmConfig config;
        std::array<infra::BoundedString::WithStorage<meshPathSize>, meshCount> meshes{};
    };

    std::optional<ArmProfile> ReadArmProfile(infra::BoundedConstString text, infra::BoundedString& problem);
    void WriteArmProfile(const ArmProfile& profile, infra::TextOutputStream& stream);
}

#endif
