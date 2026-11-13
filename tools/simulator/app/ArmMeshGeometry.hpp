#ifndef TOOLS_SIMULATOR_APP_ARM_MESH_GEOMETRY_HPP
#define TOOLS_SIMULATOR_APP_ARM_MESH_GEOMETRY_HPP

#include <QQuick3DGeometry>
#include <QtQml/qqmlregistration.h>

namespace simulator
{
    class ArmMeshGeometry
        : public QQuick3DGeometry
    {
        Q_OBJECT
        QML_ELEMENT

        Q_PROPERTY(int node READ Node WRITE SetNode NOTIFY changed)
        Q_PROPERTY(int finish READ Finish WRITE SetFinish NOTIFY changed)
        Q_PROPERTY(bool empty READ Empty NOTIFY changed)

    public:
        explicit ArmMeshGeometry(QQuick3DObject* parent = nullptr);

        int Node() const;
        void SetNode(int newNode);
        int Finish() const;
        void SetFinish(int newFinish);
        bool Empty() const;

    signals:
        void changed();

    private:
        void Rebuild();

        int node{ -1 };
        int finish{ -1 };
        bool empty{ true };
    };
}

#endif
