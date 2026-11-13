#ifndef TOOLS_SIMULATOR_APP_MESH_FILES_HPP
#define TOOLS_SIMULATOR_APP_MESH_FILES_HPP

#include <QByteArray>
#include <QString>
#include <QUrl>
#include <array>
#include <cstddef>
#include <optional>
#include <string>

namespace simulator
{
    inline constexpr std::size_t meshTriangleBudget{ 50000 };
    inline constexpr std::size_t meshNodeCount{ 7 };

    struct MeshFile
    {
        QUrl url;
        QString problem;
    };

    using MeshFiles = std::array<MeshFile, meshNodeCount>;

    std::optional<std::size_t> CountTriangles(const QByteArray& contents);
    MeshFile CheckMesh(const QString& path);
    using MeshNames = std::array<std::string, meshNodeCount>;

    MeshFiles ResolveMeshes(const MeshNames& names, const QString& directory);
    QString MeshNodeName(std::size_t node);
}

#endif
