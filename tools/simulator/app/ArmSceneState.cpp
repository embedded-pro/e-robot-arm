#include "tools/simulator/app/ArmSceneState.hpp"
#include "ui/backend/qt/QtConversions.hpp"
#include <QJSEngine>
#include <QQuaternion>
#include <QVariantMap>
#include <QVector3D>

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
            generated = model.Meshes();
            extent = model.Extent();
            emit partsChanged();
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
        selectedJoint = model.Selected() ? static_cast<int>(*model.Selected()) : -1;
        emit changed();
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

    const mesh::Mesh& ArmSceneState::MeshOf(int node, int finish) const
    {
        static const mesh::Mesh none{};
        if (node < 0 || node >= static_cast<int>(armNodeCount) || finish < 0 || finish >= static_cast<int>(finishCount))
            return none;
        return generated[static_cast<std::size_t>(node)][static_cast<std::size_t>(finish)];
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

    void ArmSceneState::RequestCameraPreset(int preset)
    {
        emit cameraPresetRequested(preset);
    }
}
