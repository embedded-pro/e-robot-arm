#pragma once

#include "tools/simulator/view_models/ArmSceneModel.hpp"
#include "ui/backend/qt/quick3d/MeshCatalog.hpp"
#include "ui/theme/Theme.hpp"
#include <QColor>
#include <QObject>
#include <QQuaternion>
#include <QStringList>
#include <QVariant>
#include <QVariantList>
#include <QVector3D>
#include <QtQml/qqmlregistration.h>
#include <array>

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
        Q_PROPERTY(QVariantList boxes READ Boxes NOTIFY boxesChanged)
        Q_PROPERTY(bool targetVisible READ TargetVisible NOTIFY changed)
        Q_PROPERTY(QVector3D targetPosition READ TargetPosition NOTIFY changed)
        Q_PROPERTY(QQuaternion targetRotation READ TargetRotation NOTIFY changed)
        Q_PROPERTY(QVariantList limitArc READ LimitArc NOTIFY limitArcChanged)
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
        Q_PROPERTY(QColor trailColor READ TrailColor NOTIFY themeChanged)
        Q_PROPERTY(QColor selectionColor READ SelectionColor NOTIFY themeChanged)
        Q_PROPERTY(QColor textColor READ TextColor NOTIFY themeChanged)
        Q_PROPERTY(QColor panelColor READ PanelColor NOTIFY themeChanged)
        Q_PROPERTY(QVariantList selectionRing READ SelectionRing NOTIFY selectionRingChanged)
        Q_PROPERTY(QString selectionText READ SelectionText NOTIFY changed)
        Q_PROPERTY(QVector3D selectionAnchor READ SelectionAnchor NOTIFY changed)
        Q_PROPERTY(QVariant frameBounds READ FrameBounds NOTIFY changed)
        Q_PROPERTY(bool orthographic READ Orthographic WRITE SetOrthographic NOTIFY overlaysChanged)
        Q_PROPERTY(bool showGhost READ ShowGhost NOTIFY overlaysChanged)
        Q_PROPERTY(bool showTrail READ ShowTrail NOTIFY overlaysChanged)
        Q_PROPERTY(bool showFrames READ ShowFrames NOTIFY overlaysChanged)
        Q_PROPERTY(bool showLimits READ ShowLimits NOTIFY overlaysChanged)
        Q_PROPERTY(bool showWorkspace READ ShowWorkspace NOTIFY overlaysChanged)
        Q_PROPERTY(bool showLegend READ ShowLegend NOTIFY overlaysChanged)

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
        QVariantList Boxes() const;
        bool TargetVisible() const;
        QVector3D TargetPosition() const;
        QQuaternion TargetRotation() const;
        QVariantList LimitArc() const;
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
        QColor TrailColor() const;
        QColor SelectionColor() const;
        QColor TextColor() const;
        QColor PanelColor() const;
        QVariantList SelectionRing() const;
        QString SelectionText() const;
        QVector3D SelectionAnchor() const;
        QVariant FrameBounds() const;

        enum class Overlay : uint8_t
        {
            ghost,
            trail,
            frames,
            limits,
            workspace,
            legend
        };

        static constexpr std::size_t overlayCount{ 6 };
        bool Shows(Overlay overlay) const;
        void SetShown(Overlay overlay, bool shown);
        bool ShowGhost() const;
        bool ShowTrail() const;
        bool ShowFrames() const;
        bool ShowLimits() const;
        bool ShowWorkspace() const;
        bool ShowLegend() const;
        bool Orthographic() const;
        void SetOrthographic(bool on);
        void RequestFocusSelected();

        Q_INVOKABLE void Pick(int joint);
        Q_INVOKABLE void ReportMeshFailure(int node, const QString& reason);
        Q_INVOKABLE void RequestCameraPreset(int preset);

    signals:
        void changed();
        void themeChanged();
        void partsChanged();
        void trailChanged();
        void boxesChanged();
        void limitArcChanged();
        void meshesChanged();
        void meshFailed(int node, const QString& reason);
        void jointPicked(int joint);
        void cameraPresetRequested(int preset);
        void selectionRingChanged();
        void overlaysChanged();
        void frameRequested();
        void focusSelectedRequested();

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
        QVariantList boxes;
        uint32_t boxesRevision{ 0 };
        bool targetVisible{ false };
        QVector3D targetPosition;
        QQuaternion targetRotation;
        QVariantList limitArc;
        uint32_t limitArcRevision{ 0 };
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
        QColor trailTone;
        QColor selectionTone;
        QColor text;
        QColor panel;
        QVariantList selectionRing;
        uint32_t selectionRingRevision{ 0 };
        QString selectionText;
        QVector3D selectionAnchor;
        QVariant frameBounds;
        std::array<bool, overlayCount> shown{ true, true, true, true, true, true };
        bool orthographic{ false };
        bool reframe{ false };
    };
}
