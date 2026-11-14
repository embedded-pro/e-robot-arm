#ifndef TOOLS_SIMULATOR_APP_ARM_SCENE_STATE_HPP
#define TOOLS_SIMULATOR_APP_ARM_SCENE_STATE_HPP

#include "tools/simulator/view_models/ArmSceneModel.hpp"
#include "ui/backend/qt/quick3d/MeshCatalog.hpp"
#include "ui/theme/Theme.hpp"
#include <QColor>
#include <QObject>
#include <QQuaternion>
#include <QStringList>
#include <QVariantList>
#include <QVector3D>
#include <QtQml/qqmlregistration.h>

class QJSEngine;
class QQmlEngine;

namespace simulator
{
    class ArmSceneState
        : public QObject
    {
        Q_OBJECT
        QML_ELEMENT
        QML_SINGLETON

        Q_PROPERTY(QVariantList measuredNodes READ MeasuredNodes NOTIFY changed)
        Q_PROPERTY(QVariantList commandedNodes READ CommandedNodes NOTIFY changed)
        Q_PROPERTY(double extent READ Extent NOTIFY partsChanged)
        Q_PROPERTY(ui::backend::qt::MeshCatalog* catalog READ Catalog CONSTANT)
        Q_PROPERTY(QString singularityWarning READ SingularityWarning NOTIFY changed)
        Q_PROPERTY(QVector3D toolPosition READ ToolPosition NOTIFY changed)
        Q_PROPERTY(QQuaternion toolRotation READ ToolRotation NOTIFY changed)
        Q_PROPERTY(QVariantList trail READ Trail NOTIFY trailChanged)
        Q_PROPERTY(QStringList meshes READ Meshes NOTIFY meshesChanged)
        Q_PROPERTY(QColor warningColor READ WarningColor NOTIFY themeChanged)
        Q_PROPERTY(QVariantList levels READ Levels NOTIFY changed)
        Q_PROPERTY(bool ghostVisible READ GhostVisible NOTIFY changed)
        Q_PROPERTY(bool anyFault READ AnyFault NOTIFY changed)
        Q_PROPERTY(int selectedJoint READ SelectedJoint NOTIFY changed)
        Q_PROPERTY(QColor backgroundColor READ BackgroundColor NOTIFY themeChanged)
        Q_PROPERTY(QColor floorColor READ FloorColor NOTIFY themeChanged)
        Q_PROPERTY(QColor gridColor READ GridColor NOTIFY themeChanged)
        Q_PROPERTY(QColor accentColor READ AccentColor NOTIFY themeChanged)
        Q_PROPERTY(QColor faultColor READ FaultColor NOTIFY themeChanged)

    public:
        static ArmSceneState* create(QQmlEngine* engine, QJSEngine* scriptEngine);
        static void Use(ArmSceneState& state);

        explicit ArmSceneState(QObject* parent);

        void Update(const ArmSceneModel& model);
        void ApplyTheme(const ui::theme::Theme& theme);

        QVariantList MeasuredNodes() const;
        QVariantList CommandedNodes() const;
        static ArmSceneState* Current();
        ui::backend::qt::MeshCatalog* Catalog();
        static QString PartKey(std::size_t node, std::size_t finish);
        double Extent() const;
        QString SingularityWarning() const;
        QVector3D ToolPosition() const;
        QQuaternion ToolRotation() const;
        QVariantList Trail() const;
        QStringList Meshes() const;
        void SetMeshes(const QStringList& urls);
        QColor WarningColor() const;
        QVariantList Levels() const;
        bool GhostVisible() const;
        bool AnyFault() const;
        int SelectedJoint() const;
        QColor BackgroundColor() const;
        QColor FloorColor() const;
        QColor GridColor() const;
        QColor AccentColor() const;
        QColor FaultColor() const;

        Q_INVOKABLE void Pick(int joint);
        Q_INVOKABLE void ReportMeshFailure(int node, const QString& reason);
        Q_INVOKABLE void RequestCameraPreset(int preset);

    signals:
        void changed();
        void themeChanged();
        void partsChanged();
        void trailChanged();
        void meshesChanged();
        void meshFailed(int node, const QString& reason);
        void jointPicked(int joint);
        void cameraPresetRequested(int preset);

    private:
        QVariantList measuredNodes;
        QVariantList commandedNodes;
        ui::backend::qt::MeshCatalog catalog;
        uint32_t meshesRevision{ 0 };
        double extent{ 100.0 };
        QString singularityWarning;
        QVector3D toolPosition;
        QQuaternion toolRotation;
        QVariantList trail;
        uint32_t trailRevision{ 0 };
        QStringList meshes;
        QColor warning;
        QVariantList levels;
        bool ghostVisible{ false };
        bool anyFault{ false };
        int selectedJoint{ -1 };
        QColor background;
        QColor floor;
        QColor grid;
        QColor accent;
        QColor fault;
    };
}

#endif
