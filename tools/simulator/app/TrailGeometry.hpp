#ifndef TOOLS_SIMULATOR_APP_TRAIL_GEOMETRY_HPP
#define TOOLS_SIMULATOR_APP_TRAIL_GEOMETRY_HPP

#include <QQuick3DGeometry>
#include <QVariantList>
#include <QtQml/qqmlregistration.h>

namespace simulator
{
    class TrailGeometry
        : public QQuick3DGeometry
    {
        Q_OBJECT
        QML_ELEMENT

        Q_PROPERTY(QVariantList points READ Points WRITE SetPoints NOTIFY pointsChanged)

    public:
        explicit TrailGeometry(QQuick3DObject* parent = nullptr);

        QVariantList Points() const;
        void SetPoints(const QVariantList& newPoints);

    signals:
        void pointsChanged();

    private:
        void Rebuild();

        QVariantList points;
    };
}

#endif
