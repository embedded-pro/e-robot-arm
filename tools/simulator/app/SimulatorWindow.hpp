#ifndef TOOLS_SIMULATOR_APP_SIMULATOR_WINDOW_HPP
#define TOOLS_SIMULATOR_APP_SIMULATOR_WINDOW_HPP

#include "tools/arm_host/ArmHost.hpp"
#include "tools/simulator/app/ArmSceneState.hpp"
#include "tools/simulator/app/EmergencyStopKey.hpp"
#include "tools/simulator/app/JointTable.hpp"
#include "tools/simulator/app/Panels.hpp"
#include "tools/simulator/app/StatusStripBar.hpp"
#include "tools/simulator/view_models/MotorControllerPresenter.hpp"
#include "tools/simulator/view_models/SimulatorPresenter.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include <QByteArray>
#include <QMainWindow>
#include <QTimer>
#include <utility>
#include <vector>

class QGroupBox;
class QLabel;
class QMenu;
class QQuickWidget;

namespace simulator
{
    class SimulatorWindow
        : public QMainWindow
    {
        Q_OBJECT

    public:
        static constexpr int framePeriodMilliseconds{ 16 };

        SimulatorWindow(arm_host::ArmHost& host, const arm::JointLimits& limits, infra::TimePoint start);

        QQuickWidget& ArmView();
        SimulatorPresenter& Presenter();
        MotorControllerPresenter& MotorController();
        void Frame();

    protected:
        void closeEvent(QCloseEvent* event) override;

    private:
        void EnableAmbientOcclusion();
        void BuildDocks();
        void BuildMenus();
        void BuildArmMenu();
        void BuildSingleStepMenu(QMenu& menu);
        void UpdateSingleStepMenu();
        void BuildSimulationMenu();
        void BuildViewMenu();
        void ConnectSignals();
        void RestoreSettings();
        QGroupBox* Group(const QString& title, QWidget* content);
        QDockWidget* Dock(const QString& title, const QString& name, QWidget* content, Qt::DockWidgetArea area);
        void ChooseTarget(int choice);
        void ConfirmRearm(std::string_view prompt);
        void SaveLog();
        void CaptureDefaultLayout();
        void ResetLayout();
        void UseTheme(bool light);
        void ShowAbout();
        void UpdateBanner(const StatusStrip& status);

        arm_host::ArmHost& host;
        ui::backend::qt::QtAppShell shell;
        StatusStripBar* strip;
        QLabel* banner;
        QQuickWidget* armView;
        JointTable* joints;
        FormPanel* operate;
        FormPanel* move;
        ScopePanel* scope;
        ArmSceneState sceneState{ nullptr };
        SimulatorPresenter presenter;
        MotorControllerPanel* motorControllerPanel;
        MotorControllerPresenter motorController;
        EventLogPanel* logPanel;
        EmergencyStopKey emergencyStopKey;
        QMenu* simulationMenu{ nullptr };
        std::vector<std::pair<QAction*, Control>> singleStepActions;
        QTimer frameTimer;
        QByteArray defaultLayout;
    };
}

#endif
