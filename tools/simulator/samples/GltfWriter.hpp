#pragma once

#include "ui/stage/Shading.hpp"
#include <array>
#include <string>
#include <vector>

namespace samples
{
    struct GltfMaterial
    {
        std::string name;
        std::array<float, 3> linearColour{};
        float metallic{ 0.0f };
        float roughness{ 0.5f };
    };

    struct GltfPrimitive
    {
        GltfMaterial material;
        ui::stage::ShadedMesh mesh;
    };

    // One node with one mesh, a primitive per material, its buffer embedded as base64: a self-contained glTF 2.0 file.
    std::string WriteGltf(const std::string& nodeName, const std::vector<GltfPrimitive>& primitives);
}
