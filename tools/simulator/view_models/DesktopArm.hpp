#ifndef TOOLS_SIMULATOR_VIEW_MODELS_DESKTOP_ARM_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_DESKTOP_ARM_HPP

#include "core/arm_model/ArmGeometry.hpp"
#include "tools/simulator/view_models/MeshKit.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace simulator
{
    enum class ArmFinish : uint8_t
    {
        aluminium,
        anodised,
        motor,
        steel,
        accent,
        rubber
    };

    constexpr std::size_t finishCount{ 6 };
    constexpr std::size_t armNodeCount{ 7 };
    using NodeMeshes = std::array<mesh::Mesh, finishCount>;
    using ArmMeshes = std::array<NodeMeshes, armNodeCount>;

    float PartThickness(const arm::ArmGeometry& geometry);
    ArmMeshes BuildDesktopArm(const arm::ArmGeometry& geometry);
}

#endif
