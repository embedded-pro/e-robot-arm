#pragma once

#include "core/arm_profile/ArmProfile.hpp"
#include "tools/simulator/samples/GltfWriter.hpp"
#include <string>
#include <vector>

namespace samples
{
    struct SampleFile
    {
        std::string name;
        std::string text;
    };

    std::vector<GltfPrimitive> SampleNodeMeshes(const arm::ArmGeometry& geometry, std::size_t node);
    arm::ArmProfile SampleProfile();
    std::vector<SampleFile> SampleArmFiles();
}
