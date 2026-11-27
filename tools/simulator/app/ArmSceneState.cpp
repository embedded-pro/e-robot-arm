#include "tools/simulator/app/ArmSceneState.hpp"
#include "ui/backend/qt/QtConversions.hpp"
#include <QJSEngine>
#include <QQuaternion>
#include <QVariantMap>
#include <QVector3D>
#include <utility>

namespace simulator
{
    namespace
    {
        ArmSceneState* instance{ nullptr };

        QVector3D Vector(const std::array<float, 3>& value)
        {
            return { value[0], value[1], value[2] };
        }

        QQuaternion Rotation(const std::array<float, 4>& value)
        {
            return { value[0], value[1], value[2], value[3] };
        }

        QVariantMap ToMap(const ScenePose& node)
        {
            return QVariantMap{ { "position", Vector(node.position) }, { "rotation", Rotation(node.rotation) } };
        }

        QVariantMap ToMap(const SceneBox& box)
        {
            QVariantList outline;
            for (const auto& point : BoxOutline(box))
                outline.append(Vector(point));
            return QVariantMap{ { "zone", box.kind == SceneBoxKind::zone },
                { "centre", Vector(box.centre) },
                { "size", Vector(box.size) },
                { "violated", box.violated },
                { "outline", outline } };
        }

        QVariantList ToList(const SceneNodes& nodes)
        {
            QVariantList list;
            for (const auto& node : nodes)
                list.append(ToMap(node));
            return list;
        }

    }

    ArmSceneState* ArmSceneState::create(QQmlEngine*, QJSEngine*)
    {
        QJSEngine::setObjectOwnership(instance, QJSEngine::CppOwnership);
        return instance;
    }

    void ArmSceneState::Use(ArmSceneState& state)
    {
        instance = &state;
    }

    ArmSceneState::ArmSceneState(QObject* parent)
        : QObject(parent)
        , measuredNodes(ToList(SceneNodes{}))
        , commandedNodes(ToList(SceneNodes{}))
    {
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            levels.append(0);
        ApplyTheme(ui::theme::Current());
    }

    void ArmSceneState::Update(const ArmSceneModel& model)
    {
        measuredNodes = ToList(model.MeasuredNodes());
        commandedNodes = ToList(model.CommandedNodes());
        if (model.MeshesRevision() != meshesRevision)
        {
            meshesRevision = model.MeshesRevision();
            for (std::size_t node = 0; node != armNodeCount; ++node)
                for (std::size_t finish = 0; finish != finishCount; ++finish)
                    catalog.Set(PartKey(node, finish), model.Meshes()[node][finish]);
            extent = model.Extent();
            emit partsChanged();
            reframe = true;
        }
        anyFault = false;
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            levels[static_cast<int>(joint)] = static_cast<int>(model.Level(joint));
            anyFault = anyFault || model.Level(joint) == ui::theme::StatusLevel::Fault;
        }
        ghostVisible = model.GhostVisible();
        singularityWarning = QString::fromStdString(model.SingularityWarning());
        toolPosition = Vector(model.ToolNode().position);
        toolRotation = Rotation(model.ToolNode().rotation);
        if (model.TrailRevision() != trailRevision)
        {
            trailRevision = model.TrailRevision();
            trail.clear();
            for (const auto& point : model.ToolTrail())
                trail.append(Vector(point));
            emit trailChanged();
        }
        if (model.BoxesRevision() != boxesRevision)
        {
            boxesRevision = model.BoxesRevision();
            boxes.clear();
            for (const auto& box : model.Boxes())
                boxes.append(ToMap(box));
            emit boxesChanged();
        }
        targetVisible = model.TargetNode().has_value();
        if (targetVisible)
        {
            targetPosition = Vector(model.TargetNode()->position);
            targetRotation = Rotation(model.TargetNode()->rotation);
        }
        if (model.LimitArcRevision() != limitArcRevision)
        {
            limitArcRevision = model.LimitArcRevision();
            limitArc.clear();
            for (const auto& point : model.LimitArc())
                limitArc.append(Vector(point));
            emit limitArcChanged();
        }
        if (model.SelectionRingRevision() != selectionRingRevision)
        {
            selectionRingRevision = model.SelectionRingRevision();
            selectionRing.clear();
            for (const auto& point : model.SelectionRing())
                selectionRing.append(Vector(point));
            emit selectionRingChanged();
        }
        selectionText = model.Selection() ? QString::fromStdString(model.Selection()->text) : QString{};
        selectionAnchor = model.Selection() ? Vector(model.Selection()->anchor) : QVector3D{};
        const auto& bounds = model.FrameBounds();
        frameBounds = QVariantMap{ { "min", Vector(bounds.min) }, { "max", Vector(bounds.max) } };
        selectedJoint = model.Selected() ? static_cast<int>(*model.Selected()) : -1;
        emit changed();
        if (std::exchange(reframe, false))
            emit frameRequested();
    }

    void ArmSceneState::ApplyTheme(const ui::theme::Theme& theme)
    {
        using ui::backend::qt::ToQt;
        using ui::theme::ColorRole;

        const auto themed = ToQt(theme.Get(ColorRole::SceneBackground));
        const bool dark = themed.lightness() < 128;
        background = dark ? QColor::fromHslF(0.6f, 0.12f, 0.30f) : themed;
        floor = dark ? QColor::fromHslF(0.6f, 0.06f, 0.42f) : ToQt(theme.Get(ColorRole::SceneSurface));
        grid = dark ? QColor::fromHslF(0.6f, 0.08f, 0.55f) : ToQt(theme.Get(ColorRole::GridMajor));
        accent = ToQt(theme.Get(ColorRole::Accent));
        fault = ToQt(theme.Get(ColorRole::Fault));
        warning = ToQt(theme.Get(ColorRole::Warning));
        trailTone = ToQt(theme.Get(ColorRole::SceneTrail));
        selectionTone = dark ? ToQt(theme.Get(ColorRole::SceneSelection)) : ToQt(theme.Get(ColorRole::SceneSelection)).darker(140);
        text = ToQt(theme.Get(ColorRole::Text));
        panel = ToQt(theme.Get(ColorRole::Surface));
        emit themeChanged();
    }

    QVariantList ArmSceneState::MeasuredNodes() const
    {
        return measuredNodes;
    }

    QVariantList ArmSceneState::CommandedNodes() const
    {
        return commandedNodes;
    }

    ArmSceneState* ArmSceneState::Current()
    {
        return instance;
    }

    ui::backend::qt::MeshCatalog* ArmSceneState::Catalog()
    {
        return &catalog;
    }

    QString ArmSceneState::PartKey(std::size_t node, std::size_t finish)
    {
        return QString{ "%1/%2" }.arg(node).arg(finish);
    }

    double ArmSceneState::Extent() const
    {
        return extent;
    }

    QString ArmSceneState::SingularityWarning() const
    {
        return singularityWarning;
    }

    QVector3D ArmSceneState::ToolPosition() const
    {
        return toolPosition;
    }

    QQuaternion ArmSceneState::ToolRotation() const
    {
        return toolRotation;
    }

    QVariantList ArmSceneState::Trail() const
    {
        return trail;
    }

    QVariantList ArmSceneState::Boxes() const
    {
        return boxes;
    }

    bool ArmSceneState::TargetVisible() const
    {
        return targetVisible;
    }

    QVector3D ArmSceneState::TargetPosition() const
    {
        return targetPosition;
    }

    QQuaternion ArmSceneState::TargetRotation() const
    {
        return targetRotation;
    }

    QVariantList ArmSceneState::LimitArc() const
    {
        return limitArc;
    }

    QColor ArmSceneState::WarningColor() const
    {
        return warning;
    }

    QVariantList ArmSceneState::Levels() const
    {
        return levels;
    }

    bool ArmSceneState::GhostVisible() const
    {
        return ghostVisible;
    }

    bool ArmSceneState::AnyFault() const
    {
        return anyFault;
    }

    int ArmSceneState::SelectedJoint() const
    {
        return selectedJoint;
    }

    QColor ArmSceneState::BackgroundColor() const
    {
        return background;
    }

    QColor ArmSceneState::FloorColor() const
    {
        return floor;
    }

    QColor ArmSceneState::GridColor() const
    {
        return grid;
    }

    QColor ArmSceneState::AccentColor() const
    {
        return accent;
    }

    QColor ArmSceneState::FaultColor() const
    {
        return fault;
    }

    QStringList ArmSceneState::Meshes() const
    {
        return meshes;
    }

    void ArmSceneState::SetMeshes(const QStringList& urls)
    {
        if (urls == meshes)
            return;
        meshes = urls;
        emit meshesChanged();
    }

    void ArmSceneState::ReportMeshFailure(int node, const QString& reason)
    {
        emit meshFailed(node, reason);
    }

    void ArmSceneState::Pick(int joint)
    {
        emit jointPicked(joint);
    }

    void ArmSceneState::GuideGrab(int joint, QVector3D hit, QVector3D origin, QVector3D direction)
    {
        GuidePointer(origin, direction);
        emit guideGrabbed(joint, hit);
    }

    void ArmSceneState::GuidePointer(QVector3D origin, QVector3D direction)
    {
        pointerOrigin = origin;
        pointerDirection = direction;
    }

    void ArmSceneState::GuideRelease()
    {
        emit guideReleased();
    }

    void ArmSceneState::DragGrab(int joint, QVector3D hit, QVector3D origin, QVector3D direction, bool follow)
    {
        GuidePointer(origin, direction);
        emit dragGrabbed(joint, hit, follow);
    }

    void ArmSceneState::DragRelease()
    {
        emit dragReleased();
    }

    void ArmSceneState::DragClear()
    {
        emit dragCleared();
    }

    void ArmSceneState::DragConfirm()
    {
        emit dragConfirmed();
    }

    void ArmSceneState::SetDrag(bool offered, const JointDrag& drag, const ControlState& move)
    {
        const auto nodes = drag.TargetNodes();
        const auto list = nodes.has_value() ? ToList(*nodes) : QVariantList{};
        const auto point = drag.TargetPoint().has_value() ? Vector(*drag.TargetPoint()) : QVector3D{};
        const auto label = ui::backend::qt::ToQt(drag.Label(move));
        if (offered == dragOffered && nodes.has_value() == dragActive && drag.Following() == dragFollowing && list == dragNodes && label == dragLabel)
            return;
        dragOffered = offered;
        dragActive = nodes.has_value();
        dragFollowing = drag.Following();
        dragNodes = list;
        dragPoint = point;
        dragLabel = label;
        emit dragChanged();
    }

    bool ArmSceneState::DragOffered() const
    {
        return dragOffered;
    }

    bool ArmSceneState::DragActive() const
    {
        return dragActive;
    }

    bool ArmSceneState::DragFollowing() const
    {
        return dragFollowing;
    }

    QVariantList ArmSceneState::DragNodes() const
    {
        return dragNodes;
    }

    QVector3D ArmSceneState::DragPoint() const
    {
        return dragPoint;
    }

    QString ArmSceneState::DragLabel() const
    {
        return dragLabel;
    }

    SceneRay ArmSceneState::GuideRay() const
    {
        return { { pointerOrigin.x(), pointerOrigin.y(), pointerOrigin.z() }, { pointerDirection.x(), pointerDirection.y(), pointerDirection.z() } };
    }

    void ArmSceneState::SetGuide(bool offered, const HandGuide& guide)
    {
        QVariantList arrow;
        if (guide.Arrow())
            arrow << Vector(guide.Arrow()->from) << Vector(guide.Arrow()->to);
        const auto point = guide.Arrow() ? Vector(guide.Arrow()->from) : QVector3D{};
        const auto label = ui::backend::qt::ToQt(guide.Label());
        if (offered == guideOffered && guide.Active() == guideActive && arrow == guideArrow && label == guideLabel)
            return;
        guideOffered = offered;
        guideActive = guide.Active();
        guideArrow = arrow;
        guidePoint = point;
        guideLabel = label;
        emit guideChanged();
    }

    bool ArmSceneState::GuideOffered() const
    {
        return guideOffered;
    }

    bool ArmSceneState::GuideActive() const
    {
        return guideActive;
    }

    QVariantList ArmSceneState::GuideArrow() const
    {
        return guideArrow;
    }

    QVector3D ArmSceneState::GuidePoint() const
    {
        return guidePoint;
    }

    QString ArmSceneState::GuideLabel() const
    {
        return guideLabel;
    }

    void ArmSceneState::RequestCameraPreset(int preset)
    {
        emit cameraPresetRequested(preset);
    }

    QColor ArmSceneState::TrailColor() const
    {
        return trailTone;
    }

    QColor ArmSceneState::SelectionColor() const
    {
        return selectionTone;
    }

    QColor ArmSceneState::TextColor() const
    {
        return text;
    }

    QColor ArmSceneState::PanelColor() const
    {
        return panel;
    }

    QVariantList ArmSceneState::SelectionRing() const
    {
        return selectionRing;
    }

    QString ArmSceneState::SelectionText() const
    {
        return selectionText;
    }

    QVector3D ArmSceneState::SelectionAnchor() const
    {
        return selectionAnchor;
    }

    QVariant ArmSceneState::FrameBounds() const
    {
        return frameBounds;
    }

    bool ArmSceneState::Shows(Overlay overlay) const
    {
        return shown[static_cast<std::size_t>(overlay)];
    }

    void ArmSceneState::SetShown(Overlay overlay, bool on)
    {
        if (shown[static_cast<std::size_t>(overlay)] == on)
            return;
        shown[static_cast<std::size_t>(overlay)] = on;
        emit overlaysChanged();
    }

    bool ArmSceneState::ShowGhost() const
    {
        return Shows(Overlay::ghost);
    }

    bool ArmSceneState::ShowTrail() const
    {
        return Shows(Overlay::trail);
    }

    bool ArmSceneState::ShowFrames() const
    {
        return Shows(Overlay::frames);
    }

    bool ArmSceneState::ShowLimits() const
    {
        return Shows(Overlay::limits);
    }

    bool ArmSceneState::ShowWorkspace() const
    {
        return Shows(Overlay::workspace);
    }

    bool ArmSceneState::ShowLegend() const
    {
        return Shows(Overlay::legend);
    }

    bool ArmSceneState::Orthographic() const
    {
        return orthographic;
    }

    void ArmSceneState::SetOrthographic(bool on)
    {
        if (orthographic == on)
            return;
        orthographic = on;
        emit overlaysChanged();
    }

    void ArmSceneState::RequestFocusSelected()
    {
        emit focusSelectedRequested();
    }
}
