#ifndef CORE_ARM_PROFILE_ARM_PROFILE_HPP
#define CORE_ARM_PROFILE_ARM_PROFILE_HPP

#include "core/arm/ArmConfig.hpp"
#include "infra/stream/OutputStream.hpp"
#include "infra/util/BoundedString.hpp"
#include <cstddef>
#include <optional>

namespace arm
{
    inline constexpr std::size_t profileNameSize{ 48 };
    inline constexpr std::size_t profileProblemSize{ 128 };
    inline constexpr int32_t profileVersion{ 1 };

    struct ArmProfile
    {
        infra::BoundedString::WithStorage<profileNameSize> name;
        ArmConfig config;
    };

    std::optional<ArmProfile> ReadArmProfile(infra::BoundedConstString text, infra::BoundedString& problem);
    void WriteArmProfile(const ArmProfile& profile, infra::TextOutputStream& stream);
}

#endif
