#ifndef TOOLS_SIMULATOR_APP_ARM_SCENE_STATE_HPP
#define TOOLS_SIMULATOR_APP_ARM_SCENE_STATE_HPP

#include "tools/simulator/view_models/ArmSceneModel.hpp"
#include "ui/theme/Theme.hpp"
#include <QColor>
#include <QObject>
#include <QVariantList>
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

        Q_PROPERTY(QVariantList measured READ Measured NOTIFY changed)
        Q_PROPERTY(QVariantList commanded READ Commanded NOTIFY changed)
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

        QVariantList Measured() const;
        QVariantList Commanded() const;
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
        Q_INVOKABLE void RequestCameraPreset(int preset);

    signals:
        void changed();
        void themeChanged();
        void jointPicked(int joint);
        void cameraPresetRequested(int preset);

    private:
        QVariantList measured;
        QVariantList commanded;
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
