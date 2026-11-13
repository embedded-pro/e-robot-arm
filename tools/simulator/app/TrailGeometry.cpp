#include "tools/simulator/app/TrailGeometry.hpp"
#include <QByteArray>
#include <QVector3D>
#include <limits>

namespace simulator
{
    TrailGeometry::TrailGeometry(QQuick3DObject* parent)
        : QQuick3DGeometry{ parent }
    {
        Rebuild();
    }

    QVariantList TrailGeometry::Points() const
    {
        return points;
    }

    void TrailGeometry::SetPoints(const QVariantList& newPoints)
    {
        points = newPoints;
        Rebuild();
        emit pointsChanged();
    }

    void TrailGeometry::Rebuild()
    {
        clear();
        setPrimitiveType(QQuick3DGeometry::PrimitiveType::LineStrip);
        setStride(3 * sizeof(float));
        addAttribute(QQuick3DGeometry::Attribute::PositionSemantic, 0, QQuick3DGeometry::Attribute::F32Type);

        QByteArray vertices;
        vertices.resize(static_cast<qsizetype>(points.size() * 3 * sizeof(float)));
        auto* data = reinterpret_cast<float*>(vertices.data());
        QVector3D lowest{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
        QVector3D highest{ -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() };
        for (const auto& value : points)
        {
            const auto point = value.value<QVector3D>();
            *data++ = point.x();
            *data++ = point.y();
            *data++ = point.z();
            lowest = QVector3D{ std::min(lowest.x(), point.x()), std::min(lowest.y(), point.y()), std::min(lowest.z(), point.z()) };
            highest = QVector3D{ std::max(highest.x(), point.x()), std::max(highest.y(), point.y()), std::max(highest.z(), point.z()) };
        }

        setVertexData(vertices);
        if (!points.isEmpty())
            setBounds(lowest, highest);
        update();
    }
}
