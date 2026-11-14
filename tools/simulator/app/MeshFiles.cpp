#include "tools/simulator/app/MeshFiles.hpp"
#include <QDir>

namespace simulator
{
    MeshFiles ResolveMeshes(const MeshNames& names, const QString& directory)
    {
        const QDir root{ directory };
        MeshFiles files{};
        for (std::size_t node = 0; node != meshNodeCount; ++node)
            if (!names[node].empty())
                files[node] = ui::backend::qt::CheckGltf(root.filePath(QString::fromStdString(names[node])), meshTriangleBudget);
        return files;
    }

    QString MeshNodeName(std::size_t node)
    {
        return node == 0 ? QString{ "Base" } : QString{ "Link %1" }.arg(node);
    }
}
