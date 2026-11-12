#include "tools/simulator/app/ArmSceneState.hpp"
#include "ui/backend/qt/QtConversions.hpp"
#include <QJSEngine>

namespace simulator
{
    namespace
    {
        ArmSceneState* instance{ nullptr };

        QVariantList ToList(const std::array<float, arm::jointCount>& values)
        {
            QVariantList list;
            for (const auto value : values)
                list.append(static_cast<double>(value));
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
        , measured(ToList({}))
        , commanded(ToList({}))
    {
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            levels.append(0);
        ApplyTheme(ui::theme::Current());
    }

    void ArmSceneState::Update(const ArmSceneModel& model)
    {
        measured = ToList(model.MeasuredDegrees());
        commanded = ToList(model.CommandedDegrees());
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

    QVariantList ArmSceneState::Measured() const
    {
        return measured;
    }

    QVariantList ArmSceneState::Commanded() const
    {
        return commanded;
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
