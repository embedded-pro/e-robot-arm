#pragma once

#include "tools/arm_host/ArmHost.hpp"
#include "tools/simulator/app/ArmSceneState.hpp"
#include "tools/simulator/app/EmergencyStopKey.hpp"
#include "tools/simulator/app/JointTable.hpp"
#include "tools/simulator/app/MeshFiles.hpp"
#include "tools/simulator/app/Panels.hpp"
#include "tools/simulator/app/StatusStripBar.hpp"
#include "tools/simulator/app/TimedQuickWidget.hpp"
#include "tools/simulator/view_models/DisturbancePresenter.hpp"
#include "tools/simulator/view_models/FrameStatistics.hpp"
#include "tools/simulator/view_models/MotorControllerPresenter.hpp"
#include "tools/simulator/view_models/MotorPresenter.hpp"
#include "tools/simulator/view_models/RecentProfiles.hpp"
#include "tools/simulator/view_models/SimulatorPresenter.hpp"
#include "tools/simulator/view_models/TuningPresenter.hpp"
#include "ui/backend/qt/QtAppShell.hpp"
#include "ui/backend/qt/update/IssueLog.hpp"
#include "ui/backend/qt/update/widgets/UpdateAssistant.hpp"
#include <QByteArray>
#include <QElapsedTimer>
#include <QMainWindow>
#include <QString>
#include <QTimer>
#include <optional>
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
        MotorPresenter& Motor();
        TuningPresenter& Tuning();
        ui::backend::qt::update::UpdateAssistant& Updates();
        void StartUpdateChecks();
        const FrameStatistics& Frames() const;
        void LoadProfileFrom(const QString& path);
        void SaveProfileTo(const QString& path);
        void Frame();

    protected:
        void closeEvent(QCloseEvent* event) override;

    private:
        void EnableAmbientOcclusion();
        void ShowFrameStatistics();
        void UpdateFrameStatistics(std::chrono::nanoseconds guiFrame);
        void BuildDocks();
        void BuildMenus();
        void BuildArmMenu();
        void BuildSingleStepMenu(QMenu& menu);
        void UpdateSingleStepMenu();
        void UpdateCommissioningItem();
        void ChooseCommissioning(bool on);
        void BuildSimulationMenu();
        void BuildViewMenu();
        void ConnectSignals();
        void ConnectJog();
        void ConnectTeach();
        void RestoreSettings();
        QGroupBox* Group(const QString& title, QWidget* content);
        QDockWidget* Dock(const QString& title, const QString& name, QWidget* content, Qt::DockWidgetArea area);
        QDockWidget* ConfigurationDock(const QString& title, const QString& name, ConfigurationPanel* panel, QDockWidget* tabbedWith);
        void ChooseTarget(int choice);
        void ConfirmRearm(std::string_view prompt);
        void ConfirmDisable(std::string_view prompt);
        void ConfirmControllerDisable(std::string_view prompt);
        void SaveLog();
        void OpenProfile();
        void RememberProfile(const QString& path);
        void BuildRecentMenu();
        void SaveProfile();
        void CaptureDefaultLayout();
        void ResetLayout();
        void UseTheme(bool light);
        void ShowAbout();
        void UpdateBanner(const StatusStrip& status);
        void UpdateVirtualMotors(const arm_host::ArmSnapshot& snapshot);
        void UpdateDisturbances(const arm_host::ArmSnapshot& snapshot);
        void ExportTuningRun();
        void ChooseVirtualArm();
        void ApplyMeshes(const MeshNames& names, const QString& directory);
        void NoteMesh(int node, const QString& problem);
        void ShowNewestLog(std::size_t count);
        void BuildHelpMenu();
        void ConnectUpdates();

        arm_host::ArmHost& host;
        ui::backend::qt::QtAppShell shell;
        StatusStripBar* strip;
        QLabel* banner;
        FrameStatistics frameStatistics{ std::chrono::nanoseconds{ 0 } };
        TimedQuickWidget* armView;
        JointTable* joints;
        ToolPanel* tool;
        FormPanel* operate;
        FormPanel* move;
        ScopePanel* scope;
        ArmSceneState sceneState{ nullptr };
        SimulatorPresenter presenter;
        ConfigurationPanel* motorControllerPanel;
        MotorControllerPresenter motorController;
        ConfigurationPanel* motorPanel;
        MotorPresenter motor;
        ArmControllersPanel* armControllersPanel;
        ArmControllersPresenter armControllers;
        JogPanel* jogPanel;
        JogPresenter jog;
        TeachPanel* teachPanel;
        TeachPresenter teach;
        ProfilePanel* profilePanel;
        VirtualMotorsPanel* virtualMotorsPanel;
        DisturbancesPanel* disturbancesPanel;
        DisturbancePresenter disturbances;
        QDockWidget* disturbancesDock{ nullptr };
        std::optional<bool> disturbancesShown;
        TuningPanel* tuningPanel;
        TuningPresenter tuning;
        QLabel* frameReadout;
        QElapsedTimer clock;
        QMenu* faultMenu{ nullptr };
        QDockWidget* virtualMotorsDock{ nullptr };
        std::optional<bool> virtualMotorsShown;
        QString profilePath;
        RecentProfiles recentProfiles;
        QMenu* recentMenu{ nullptr };
        MeshNames meshNames{};
        QString meshDirectory;
        std::optional<std::pair<MeshNames, QString>> pendingMeshes;
        EventLogPanel* logPanel;
        EmergencyStopKey emergencyStopKey;
        QMenu* simulationMenu{ nullptr };
        std::vector<std::pair<QAction*, Control>> singleStepActions;
        QAction* commissioningAction{ nullptr };
        QTimer frameTimer;
        QByteArray defaultLayout;
        ui::backend::qt::update::IssueLog issueLog;
        ui::backend::qt::update::UpdateAssistant updates;
        QAction* automaticUpdatesAction{ nullptr };
    };
}
