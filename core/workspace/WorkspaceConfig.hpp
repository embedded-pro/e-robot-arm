#ifndef CORE_WORKSPACE_WORKSPACE_CONFIG_HPP
#define CORE_WORKSPACE_WORKSPACE_CONFIG_HPP

#include "numerical/math/Geometry3D.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace arm
{
    inline constexpr std::size_t workspaceCapsules{ 7 };
    inline constexpr std::size_t maxForbiddenZones{ 8 };
    inline constexpr std::size_t maxAllowedLinkPairs{ 16 };

    struct WorkspaceBox
    {
        math::Vector3<float> min{};
        math::Vector3<float> max{};
    };

    using LinkPair = std::array<uint8_t, 2>;

    struct WorkspaceConfig
    {
        std::optional<WorkspaceBox> workspace;
        std::array<WorkspaceBox, maxForbiddenZones> forbiddenZones{};
        uint8_t forbiddenZoneCount{ 0 };
        std::array<float, workspaceCapsules> linkRadii{ 0.04f, 0.04f, 0.04f, 0.04f, 0.04f, 0.04f, 0.04f };
        float selfCollisionMargin{ 0.01f };
        std::array<LinkPair, maxAllowedLinkPairs> allowedLinkPairs{};
        uint8_t allowedLinkPairCount{ 0 };
    };
}

#endif
