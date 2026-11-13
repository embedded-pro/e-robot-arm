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

        QVariantList ToList(const SceneNodes& nodes)
        {
            QVariantList list;
            for (const auto& node : nodes)
                list.append(QVariantMap{ { "position", Vector(node.position) }, { "rotation", Rotation(node.rotation) } });
            return list;
        }

        QVariantList ToList(const SceneParts& parts)
        {
            QVariantList nodes;
            for (const auto& nodeParts : parts)
            {
                QVariantList list;
                for (const auto& part : nodeParts)
                    list.append(QVariantMap{ { "joint", part.joint }, { "source", part.shape == PartShape::cylinder ? "#Cylinder" : "#Cube" },
                        { "material", static_cast<int>(part.material) }, { "position", Vector(part.centre) }, { "rotation", Rotation(part.rotation) },
                        { "scale", Vector(part.scale) } });
                nodes.append(QVariant{ list });
            }
            return nodes;
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
        , parts(ToList(SceneParts{}))
    {
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            levels.append(0);
        ApplyTheme(ui::theme::Current());
    }

    void ArmSceneState::Update(const ArmSceneModel& model)
    {
        measuredNodes = ToList(model.MeasuredNodes());
        commandedNodes = ToList(model.CommandedNodes());
        if (model.PartsRevision() != partsRevision)
        {
            partsRevision = model.PartsRevision();
            parts = ToList(model.Parts());
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
        selectedJoint = model.Selected() ? static_cast<int>(*model.Selected()) : -1;
        emit changed();
    }

    void ArmSceneState::ApplyTheme(const ui::theme::Theme& theme)
    {
        using ui::backend::qt::ToQt;
        using ui::theme::ColorRole;

        background = ToQt(theme.Get(ColorRole::SceneBackground));
        const bool dark = background.lightness() < 128;
        floor = dark ? QColor::fromHslF(0.0f, 0.0f, 0.30f) : ToQt(theme.Get(ColorRole::SceneSurface));
        grid = dark ? QColor::fromHslF(0.0f, 0.0f, 0.42f) : ToQt(theme.Get(ColorRole::GridMajor));
        accent = ToQt(theme.Get(ColorRole::Accent));
        fault = ToQt(theme.Get(ColorRole::Fault));
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

    QVariantList ArmSceneState::Parts() const
    {
        return parts;
    }

    double ArmSceneState::Extent() const
    {
        return extent;
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

    void ArmSceneState::Pick(int joint)
    {
        emit jointPicked(joint);
    }

    void ArmSceneState::RequestCameraPreset(int preset)
    {
        emit cameraPresetRequested(preset);
    }
}
