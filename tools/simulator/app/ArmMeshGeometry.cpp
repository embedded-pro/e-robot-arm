#include "tools/simulator/app/ArmMeshGeometry.hpp"
#include "tools/simulator/app/ArmSceneState.hpp"
#include <QByteArray>
#include <QVector3D>
#include <cstring>

namespace simulator
{
    ArmMeshGeometry::ArmMeshGeometry(QQuick3DObject* parent)
        : QQuick3DGeometry{ parent }
    {
        if (auto* state = ArmSceneState::Current())
            connect(state, &ArmSceneState::partsChanged, this, &ArmMeshGeometry::Rebuild);
    }

    int ArmMeshGeometry::Node() const
    {
        return node;
    }

    void ArmMeshGeometry::SetNode(int newNode)
    {
        if (newNode == node)
            return;
        node = newNode;
        Rebuild();
    }

    int ArmMeshGeometry::Finish() const
    {
        return finish;
    }

    void ArmMeshGeometry::SetFinish(int newFinish)
    {
        if (newFinish == finish)
            return;
        finish = newFinish;
        Rebuild();
    }

    bool ArmMeshGeometry::Empty() const
    {
        return empty;
    }

    void ArmMeshGeometry::Rebuild()
    {
        const auto* state = ArmSceneState::Current();
        static const mesh::Mesh none{};
        const auto& source = state != nullptr ? state->MeshOf(node, finish) : none;

        clear();
        setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);
        setStride(sizeof(mesh::Vertex));
        addAttribute(QQuick3DGeometry::Attribute::PositionSemantic, 0, QQuick3DGeometry::Attribute::F32Type);
        addAttribute(QQuick3DGeometry::Attribute::NormalSemantic, sizeof(mesh::Vec3), QQuick3DGeometry::Attribute::F32Type);
        addAttribute(QQuick3DGeometry::Attribute::IndexSemantic, 0, QQuick3DGeometry::Attribute::U32Type);

        const auto& vertices = source.Vertices();
        const auto& indices = source.Indices();
        QByteArray vertexData(static_cast<qsizetype>(vertices.size() * sizeof(mesh::Vertex)), Qt::Uninitialized);
        std::memcpy(vertexData.data(), vertices.data(), static_cast<std::size_t>(vertexData.size()));
        QByteArray indexData(static_cast<qsizetype>(indices.size() * sizeof(uint32_t)), Qt::Uninitialized);
        std::memcpy(indexData.data(), indices.data(), static_cast<std::size_t>(indexData.size()));
        setVertexData(vertexData);
        setIndexData(indexData);

        if (!source.Empty())
        {
            const auto bounds = source.Extent();
            setBounds({ bounds.lowest.x, bounds.lowest.y, bounds.lowest.z }, { bounds.highest.x, bounds.highest.y, bounds.highest.z });
        }
        empty = source.Empty();
        update();
        emit changed();
    }
}
