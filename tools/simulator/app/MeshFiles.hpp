#ifndef TOOLS_SIMULATOR_APP_MESH_FILES_HPP
#define TOOLS_SIMULATOR_APP_MESH_FILES_HPP

#include "ui/backend/qt/quick3d/GltfInspection.hpp"
#include <QString>
#include <array>
#include <cstddef>
#include <string>

namespace simulator
{
    inline constexpr std::size_t meshTriangleBudget{ 50000 };
    inline constexpr std::size_t meshNodeCount{ 7 };

    using MeshFile = ui::backend::qt::GltfCheck;
    using MeshFiles = std::array<MeshFile, meshNodeCount>;
    using MeshNames = std::array<std::string, meshNodeCount>;

    MeshFiles ResolveMeshes(const MeshNames& names, const QString& directory);
    QString MeshNodeName(std::size_t node);
}

#endif
