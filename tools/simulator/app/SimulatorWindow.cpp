#include "tools/simulator/app/SimulatorWindow.hpp"
#include "tools/arm_host/AdapterTargetLink.hpp"
#include "tools/arm_host/ProfileFile.hpp"
#include "tools/simulator/app/AdapterDialog.hpp"
#include "tools/simulator/view_models/AdapterChoices.hpp"
#include "ui/backend/qt/QtConversions.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDesktopServices>
#include <QDir>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QInputDialog>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPointer>
#include <QProcess>
#include <QPushButton>
#include <QQmlEngine>
#include <QQuickItem>
#include <QQuickWidget>
#include <QQuickWindow>
#include <QScrollArea>
#include <QSettings>
#include <QStatusBar>
#include <QToolBar>
#include <QUrl>
#include <QVBoxLayout>
#include <QtGlobal>
#include <array>
#include <cstdio>
#include <fstream>
#include <utility>

namespace simulator
{
    namespace
    {
        constexpr ui::shell::ShellSpec shellSpec{ "e-robot-arm Simulator", { 1600.0f, 900.0f }, 420.0f, {}, "Ready" };

        ui::backend::qt::update::UpdateProduct Product()
        {
            const QString version{ E_ROBOT_ARM_VERSION };
            return { "e-robot-arm Simulator", "Simulator", version, QUrl{ "https://api.github.com/repos/embedded-pro/e-robot-arm/releases/latest" },
                QUrl{ "https://github.com/embedded-pro/e-robot-arm/releases" }, QUrl{ "https://github.com/embedded-pro/e-robot-arm/issues/new" },
                QByteArray{ "e-robot-arm-simulator/" } + version.toUtf8(), "Disable the arm" };
        }

        ui::backend::qt::update::IssueSeverity SeverityOf(LogLevel level)
        {
            switch (level)
            {
                case LogLevel::fault:
                    return ui::backend::qt::update::IssueSeverity::error;
                case LogLevel::warning:
                    return ui::backend::qt::update::IssueSeverity::warning;
                default:
                    return ui::backend::qt::update::IssueSeverity::info;
            }
        }

        LogLevel LevelOf(ui::backend::qt::update::IssueSeverity severity)
        {
            return severity == ui::backend::qt::update::IssueSeverity::info ? LogLevel::info : LogLevel::warning;
        }

        QSettings Settings()
        {
            return QSettings{ "embedded-pro", "e-robot-arm Simulator" };
        }

        QString GraphicsApiName(QSGRendererInterface::GraphicsApi api)
        {
            switch (api)
            {
                case QSGRendererInterface::OpenGL:
                    return "OpenGL";
                case QSGRendererInterface::Direct3D11:
                    return "Direct3D 11";
                case QSGRendererInterface::Vulkan:
                    return "Vulkan";
                case QSGRendererInterface::Metal:
                    return "Metal";
                default:
                    return "other";
            }
        }
    }

    SimulatorWindow::SimulatorWindow(arm_host::ArmHost& host, const arm::JointLimits& limits, infra::TimePoint start, ui::backend::qt::update::IssueLog& issueLog, StartOptions options)
        : host{ host }
        , shell{ *this, shellSpec }
        , strip{ new StatusStripBar{ this } }
        , banner{ new QLabel{ this } }
        , armView{ new TimedQuickWidget{ [this](std::chrono::nanoseconds duration)
              {
                  frameStatistics.AddRender(duration);
              },
              this } }
        , joints{ new JointTable{ this } }
        , tool{ new ToolPanel{ this } }
        , operate{ new FormPanel{ this } }
        , move{ new FormPanel{ this } }
        , scope{ new ScopePanel{ this } }
        , presenter{ host, shell, operate->Form(), move->Form(), limits, start }
        , motorControllerPanel{ new ConfigurationPanel{ this } }
        , motorController{ host, motorControllerPanel->Form() }
        , motorPanel{ new ConfigurationPanel{ this } }
        , motor{ host, motorPanel->Form() }
        , armControllersPanel{ new ArmControllersPanel{ this } }
        , armControllers{ [this](const arm_host::ArmCommand& command)
            {
                presenter.Request(command);
            } }
        , jogPanel{ new JogPanel{ this } }
        , jog{ [this](const arm_host::ArmCommand& command)
            {
                this->host.Post(command);
            } }
        , teachPanel{ new TeachPanel{ this } }
        , teach{ [this](const arm_host::ArmCommand& command)
            {
                presenter.Request(command);
            } }
        , profilePanel{ new ProfilePanel{ this } }
        , virtualMotorsPanel{ new VirtualMotorsPanel{ this } }
        , disturbancesPanel{ new DisturbancesPanel{ this } }
        , disturbances{ disturbancesPanel->Form(), [this](const arm_host::ArmCommand& command)
            {
                presenter.Request(command);
            } }
        , tuningPanel{ new TuningPanel{ this } }
        , tuning{ tuningPanel->Form(), host }
        , frameReadout{ new QLabel{ this } }
        , recentProfiles{ [](const std::string& path)
            {
                return QFileInfo::exists(QString::fromStdString(path));
            } }
        , logPanel{ new EventLogPanel{ presenter.Log(), this } }
        , emergencyStopKey{ *QCoreApplication::instance(), [this]()
            {
                presenter.EmergencyStop();
            } }
        , issueLog{ issueLog }
        , options{ options }
        , updates{ Product(), *this }
    {
        setMinimumSize(1280, 720);
        ArmSceneState::Use(sceneState);
        armView->setResizeMode(QQuickWidget::SizeRootObjectToView);
        armView->setAccessibleName("3-D arm view");
        armView->setFocusPolicy(Qt::StrongFocus);
        armView->engine()->addImportPath(":/");
        armView->setSource(QUrl{ "qrc:/Simulator/qml/ArmScene.qml" });
        LogSceneStatus();
        EnableAmbientOcclusion();
        ShowFrameStatistics();
        auto* content = new QWidget{ this };
        auto* contentLayout = new QVBoxLayout{ content };
        contentLayout->setContentsMargins(0, 0, 0, 0);
        contentLayout->setSpacing(0);
        contentLayout->addWidget(&updates.Banner());
        contentLayout->addWidget(armView, 1);
        shell.SetContent(content);

        addToolBar(Qt::TopToolBarArea, strip);
        addToolBarBreak(Qt::TopToolBarArea);
        auto* bannerBar = addToolBar("Real arm");
        bannerBar->setObjectName("realArmBanner");
        bannerBar->setMovable(false);
        bannerBar->toggleViewAction()->setEnabled(false);
        bannerBar->addWidget(banner);
        banner->setAccessibleName("Real arm warning");
        ui::backend::qt::StyleStatusLabel(*banner, ui::theme::StatusLevel::Warning);

        BuildDocks();
        BuildMenus();
        ConnectSignals();
        ConnectUpdates();
        RestoreSettings();
        QTimer::singleShot(0, this, &SimulatorWindow::CaptureDefaultLayout);

        frameTimer.setInterval(framePeriodMilliseconds);
        connect(&frameTimer, &QTimer::timeout, this, &SimulatorWindow::Frame);
        frameTimer.start();
        Frame();
    }

    void SimulatorWindow::EnableAmbientOcclusion()
    {
        auto* root = armView->rootObject();
        auto* studio = root != nullptr ? root->findChild<QObject*>("studio") : nullptr;
        auto* view = studio != nullptr ? studio->property("view").value<QObject*>() : nullptr;
        auto* environment = view != nullptr ? view->property("environment").value<QObject*>() : nullptr;
        if (environment != nullptr && environment->metaObject()->indexOfProperty("aoEnabled") >= 0)
            environment->setProperty("aoEnabled", true);
    }

    void SimulatorWindow::LogSceneStatus()
    {
        if (armView->status() == QQuickWidget::Ready)
        {
            issueLog.Write(ui::backend::qt::update::IssueSeverity::info, "Arm scene: ready");
            return;
        }

        QStringList errors;
        for (const auto& error : armView->errors())
            errors.append(error.toString());
        issueLog.Write(ui::backend::qt::update::IssueSeverity::error, "Arm scene: not loaded: " + errors.join("; "));
    }

    void SimulatorWindow::LogFirstFrame()
    {
        if (firstFrameLogged || !isVisible() || armView->quickWindow() == nullptr)
            return;

        firstFrameLogged = true;
        issueLog.Write(ui::backend::qt::update::IssueSeverity::info,
            "First frame rendered with " + GraphicsApiName(armView->quickWindow()->rendererInterface()->graphicsApi()));
    }

    void SimulatorWindow::ShowFrameStatistics()
    {
        frameReadout->setAccessibleName("Frame statistics");
        frameReadout->setText(QString::fromStdString(frameStatistics.Text()));
        ui::backend::qt::StyleStatusLabel(*frameReadout, frameStatistics.Level());
        statusBar()->addPermanentWidget(frameReadout);
        clock.start();
    }

    void SimulatorWindow::UpdateFrameStatistics(std::chrono::nanoseconds guiFrame)
    {
        frameStatistics.AddGuiFrame(guiFrame);
        if (!frameStatistics.Advance(std::chrono::nanoseconds{ clock.nsecsElapsed() }))
            return;

        frameReadout->setText(QString::fromStdString(frameStatistics.Text()));
        ui::backend::qt::StyleStatusLabel(*frameReadout, frameStatistics.Level());
#ifndef NDEBUG
        std::fprintf(stderr, "[frames] %s\n", frameStatistics.Text().c_str());
#endif
        // Each second while starting, then each minute: shows a slow or stalled GUI thread in the log.
        if (++frameSummaries <= 30 || frameSummaries % 60 == 0)
            issueLog.Write(ui::backend::qt::update::IssueSeverity::info, "Frames: " + QString::fromStdString(frameStatistics.Text()));
    }

    const FrameStatistics& SimulatorWindow::Frames() const
    {
        return frameStatistics;
    }

    QQuickWidget& SimulatorWindow::ArmView()
    {
        return *armView;
    }

    SimulatorPresenter& SimulatorWindow::Presenter()
    {
        return presenter;
    }

    MotorControllerPresenter& SimulatorWindow::MotorController()
    {
        return motorController;
    }

    MotorPresenter& SimulatorWindow::Motor()
    {
        return motor;
    }

    TuningPresenter& SimulatorWindow::Tuning()
    {
        return tuning;
    }

    void SimulatorWindow::Frame()
    {
        QElapsedTimer guiFrame;
        guiFrame.start();
        const auto snapshot = host.Snapshot();
        const auto events = host.TakeEvents();
        const auto trace = host.TakeTrace();

        presenter.Refresh(snapshot, events);
        const auto traceGaps = presenter.NoteTraceGaps(snapshot, trace);
        if (pendingMeshes && std::ranges::any_of(events, [](const arm_host::ArmEvent& event)
                                 {
                                     return event.kind == arm_host::ArmEventKind::profileLoaded;
                                 }))
        {
            ApplyMeshes(pendingMeshes->first, pendingMeshes->second);
            pendingMeshes.reset();
        }
        const auto& status = presenter.Status();
        strip->Update(status, presenter.State(Control::emergencyStop), presenter.State(Control::target), snapshot.target.kind);
        UpdateBanner(status);

        joints->Update(presenter.Rows());
        joints->Select(presenter.Scene().Selected());
        tool->Update(presenter.Tool());
        operate->SetHint(presenter.OperateHint());
        move->SetHint(presenter.MoveHint());

        if (const auto selected = presenter.Scene().Selected())
        {
            motorController.SelectJoint(*selected);
            motor.SelectJoint(*selected);
            disturbances.SelectJoint(*selected);
            tuning.SelectJoint(*selected);
        }
        motorController.Refresh(snapshot, events);
        motorControllerPanel->SetState(motorController.StateText(), motorController.StateLevel());
        motorControllerPanel->SetHint(motorController.Hint());
        motor.Refresh(snapshot, events);
        motorPanel->SetState(motor.StateText(), motor.StateLevel());
        motorPanel->SetHint(motor.Hint());
        armControllers.Refresh(snapshot);
        armControllersPanel->Update(armControllers);
        jog.Refresh(snapshot, std::chrono::steady_clock::now());
        jogPanel->Update(jog);
        teach.Refresh(snapshot);
        teachPanel->Update(teach);
        profilePanel->Update(DescribeProfile(snapshot), profilePath);
        UpdateVirtualMotors(snapshot);
        UpdateDisturbances(snapshot);
        tuning.Refresh(snapshot, events, trace);
        tuningPanel->Update(tuning);

        scope->Add(trace);
        if (std::ranges::any_of(events, [](const arm_host::ArmEvent& event)
                {
                    return event.kind == arm_host::ArmEventKind::tripped || event.kind == arm_host::ArmEventKind::protectiveStop;
                }))
            scope->TriggerOnTrip();
        scope->Repaint();

        ShowNewestLog(events.size() + traceGaps);
        updates.SetBlocked(DrivesJoints(snapshot));
        sceneState.Update(presenter.Scene());
        simulationMenu->setEnabled(snapshot.targetReady && snapshot.disturbances.has_value());
        faultMenu->setEnabled(snapshot.disturbances.has_value() && snapshot.disturbances->injectFault);
        UpdateFrameStatistics(std::chrono::nanoseconds{ guiFrame.nsecsElapsed() });
        LogFirstFrame();
    }

    void SimulatorWindow::ApplyMeshes(const MeshNames& names, const QString& directory)
    {
        meshNames = names;
        meshDirectory = directory;
        const auto files = ResolveMeshes(names, directory);
        QStringList urls;
        for (std::size_t node = 0; node != files.size(); ++node)
        {
            urls << files[node].url.toString();
            if (!files[node].problem.isEmpty())
                NoteMesh(static_cast<int>(node), files[node].problem);
        }
        sceneState.SetMeshes(urls);
    }

    void SimulatorWindow::NoteMesh(int node, const QString& problem)
    {
        const auto joint = node == 0 ? arm_host::noJoint : static_cast<std::size_t>(node - 1);
        presenter.Note(LogLevel::warning, joint, (MeshNodeName(static_cast<std::size_t>(node)) + " mesh not loaded: " + problem).toStdString());
        ShowNewestLog(1);
    }

    void SimulatorWindow::UpdateVirtualMotors(const arm_host::ArmSnapshot& snapshot)
    {
        const auto view = ViewVirtualMotors(snapshot);
        virtualMotorsPanel->Update(view);
        if (virtualMotorsShown == view.visible)
            return;

        virtualMotorsShown = view.visible;
        virtualMotorsDock->toggleViewAction()->setEnabled(view.visible);
        virtualMotorsDock->setVisible(view.visible);
        if (view.visible)
            virtualMotorsDock->raise();
    }

    void SimulatorWindow::UpdateDisturbances(const arm_host::ArmSnapshot& snapshot)
    {
        disturbances.Refresh(snapshot);
        disturbancesPanel->SetActive(disturbances.Active());
        disturbancesPanel->SetHint(disturbances.Hint());
        if (disturbancesShown == disturbances.Visible())
            return;

        disturbancesShown = disturbances.Visible();
        disturbancesDock->toggleViewAction()->setEnabled(*disturbancesShown);
        disturbancesDock->setVisible(*disturbancesShown);
    }

    void SimulatorWindow::UpdateBanner(const StatusStrip& status)
    {
        banner->parentWidget()->setVisible(status.realArm);
        banner->setText(ui::backend::qt::ToQt("  " + status.target + " — commands move real hardware"));
        setWindowTitle(ui::backend::qt::ToQt(WindowTitle(host.Snapshot().profileName, status.target)));
    }

    void SimulatorWindow::BuildDocks()
    {
        setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowTabbedDocks);
        auto* column = new QWidget{ this };
        auto* layout = new QVBoxLayout{ column };
        layout->setContentsMargins(8, 8, 8, 8);
        layout->setSpacing(16);
        layout->addWidget(Group("Joints", joints));
        layout->addWidget(Group("Tool", tool));
        layout->addWidget(Group("Operate", operate));
        layout->addWidget(Group("Move", move));
        layout->addStretch();

        auto* scroll = new QScrollArea{ this };
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setWidget(column);
        scroll->setMinimumWidth(480);
        auto* armDock = Dock("Arm", "armDock", scroll, Qt::RightDockWidgetArea);

        const auto showing = [this](auto& panelPresenter)
        {
            return [&panelPresenter](bool visible)
            {
                panelPresenter.SetShown(visible);
            };
        };
        connect(ConfigurationDock("Motor Controller", "motorControllerDock", motorControllerPanel, armDock), &QDockWidget::visibilityChanged, this,
            showing(motorController));
        connect(ConfigurationDock("Motor", "motorDock", motorPanel, armDock), &QDockWidget::visibilityChanged, this, showing(motor));
        tabifyDockWidget(armDock, Dock("Arm Controllers", "armControllersDock", armControllersPanel, Qt::RightDockWidgetArea));
        auto* jogDock = Dock("Jog", "jogDock", jogPanel, Qt::RightDockWidgetArea);
        tabifyDockWidget(armDock, jogDock);
        tabifyDockWidget(jogDock, Dock("Teach Points", "teachPointsDock", teachPanel, Qt::RightDockWidgetArea));
        connect(jogDock, &QDockWidget::visibilityChanged, this, [this](bool visible)
            {
                if (!visible)
                    jog.Release();
            });
        motorController.SetShown(false);
        motor.SetShown(false);
        auto* profileDock = Dock("Profile", "profileDock", profilePanel, Qt::RightDockWidgetArea);
        tabifyDockWidget(armDock, profileDock);
        virtualMotorsDock = Dock("Virtual Motors", "virtualMotorsDock", virtualMotorsPanel, Qt::RightDockWidgetArea);
        tabifyDockWidget(profileDock, virtualMotorsDock);
        disturbancesDock = Dock("Disturbances", "disturbancesDock", disturbancesPanel, Qt::RightDockWidgetArea);
        tabifyDockWidget(virtualMotorsDock, disturbancesDock);
        armDock->raise();
        auto* scopeDock = Dock("Scope", "scopeDock", scope, Qt::BottomDockWidgetArea);
        tabifyDockWidget(scopeDock, Dock("Tuning", "tuningDock", tuningPanel, Qt::BottomDockWidgetArea));
        scopeDock->raise();
        Dock("Event Log", "logDock", logPanel, Qt::BottomDockWidgetArea);
    }

    QDockWidget* SimulatorWindow::ConfigurationDock(const QString& title, const QString& name, ConfigurationPanel* panel, QDockWidget* tabbedWith)
    {
        auto* panelScroll = new QScrollArea{ this };
        panelScroll->setWidgetResizable(true);
        panelScroll->setFrameShape(QFrame::NoFrame);
        panelScroll->setWidget(panel);
        auto* dock = Dock(title, name, panelScroll, Qt::RightDockWidgetArea);
        tabifyDockWidget(tabbedWith, dock);
        return dock;
    }

    QGroupBox* SimulatorWindow::Group(const QString& title, QWidget* content)
    {
        auto* group = new QGroupBox{ title, this };
        auto* layout = new QVBoxLayout{ group };
        layout->setContentsMargins(4, 8, 4, 4);
        layout->addWidget(content);
        return group;
    }

    QDockWidget* SimulatorWindow::Dock(const QString& title, const QString& name, QWidget* content, Qt::DockWidgetArea area)
    {
        auto* dock = new QDockWidget{ title, this };
        dock->setObjectName(name);
        dock->setWidget(content);
        dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
        addDockWidget(area, dock);
        return dock;
    }

    void SimulatorWindow::BuildMenus()
    {
        auto* file = menuBar()->addMenu("&File");
        file->addAction("&Open Profile…", QKeySequence::Open, this, &SimulatorWindow::OpenProfile);
        recentMenu = file->addMenu("Open &Recent");
        recentMenu->setToolTipsVisible(true);
        BuildRecentMenu();
        file->addAction("Save &Profile…", QKeySequence{ "Ctrl+Shift+S" }, this, &SimulatorWindow::SaveProfile);
        file->addSeparator();
        file->addAction("&Save Log…", QKeySequence::Save, this, &SimulatorWindow::SaveLog);
        file->addSeparator();
        file->addAction("&Quit", QKeySequence::Quit, this, &QWidget::close);

        BuildArmMenu();
        BuildSimulationMenu();
        BuildViewMenu();

        BuildHelpMenu();
    }

    void SimulatorWindow::BuildHelpMenu()
    {
        auto* help = menuBar()->addMenu("&Help");
        help->addAction("Check for &Updates…", this, [this]()
            {
                updates.Check(true);
            });
        automaticUpdatesAction = help->addAction("Check for Updates &Automatically");
        automaticUpdatesAction->setCheckable(true);
        automaticUpdatesAction->setChecked(updates.Automatic());
        connect(automaticUpdatesAction, &QAction::toggled, this, [this](bool automatic)
            {
                updates.SetAutomatic(automatic);
            });
        help->addAction("&Report Issue…", this, [this]()
            {
                ui::backend::qt::update::OpenIssueReport(Product(), issueLog, this);
            });
        help->addAction("Open &Log Folder", this, [this]()
            {
                QDesktopServices::openUrl(QUrl::fromLocalFile(issueLog.Directory()));
            });
        help->addSeparator();
        help->addAction("&About", this, &SimulatorWindow::ShowAbout);
    }

    void SimulatorWindow::ConnectUpdates()
    {
        connect(&updates, &ui::backend::qt::update::UpdateAssistant::Report, this, [this](ui::backend::qt::update::IssueSeverity severity, const QString& message)
            {
                presenter.Note(LevelOf(severity), arm_host::noJoint, message.toStdString());
                ShowNewestLog(1);
            });
        connect(&updates, &ui::backend::qt::update::UpdateAssistant::Checking, this, [this]()
            {
                statusBar()->showMessage("Checking for updates…", 5000);
            });
        connect(&updates, &ui::backend::qt::update::UpdateAssistant::RestartRequested, this, [this]()
            {
                close();
                QApplication::quit();
            });
        connect(&updates, &ui::backend::qt::update::UpdateAssistant::RelaunchRequested, this, [this]()
            {
                if (!QProcess::startDetached(QCoreApplication::applicationFilePath(), {}))
                {
                    presenter.Note(LogLevel::warning, arm_host::noJoint, "Could not start the new version; restart the Simulator to use it");
                    ShowNewestLog(1);
                    return;
                }
                close();
                QApplication::quit();
            });
    }

    void SimulatorWindow::ShowNewestLog(std::size_t count)
    {
        logPanel->AppendNewest(count);
        const auto& log = presenter.Log();
        const auto& lines = log.Lines();
        for (auto index = lines.size() - std::min(count, lines.size()); index < lines.size(); ++index)
            issueLog.Write(SeverityOf(lines[index].level), QString::fromStdString(log.Format(lines[index])));
    }

    ui::backend::qt::update::UpdateAssistant& SimulatorWindow::Updates()
    {
        return updates;
    }

    void SimulatorWindow::StartUpdateChecks()
    {
        updates.StartAutomaticChecks();
    }

    void SimulatorWindow::BuildArmMenu()
    {
        auto* armMenu = menuBar()->addMenu("&Arm");
        armMenu->addAction("&Emergency Stop\tEsc", this, [this]()
            {
                presenter.EmergencyStop();
            });

        armMenu->addSeparator();
        auto* moveAction = armMenu->addAction("&Move", this, [this]()
            {
                presenter.MoveForm().TriggerAction(MoveAction::move);
            });
        moveAction->setShortcut(QKeySequence{ "Ctrl+Return" });
        auto* abortAction = armMenu->addAction("&Abort Move", this, [this]()
            {
                presenter.MoveForm().TriggerAction(MoveAction::abort);
            });
        abortAction->setShortcut(QKeySequence{ "Ctrl+." });

        armMenu->addSeparator();
        commissioningAction = armMenu->addAction("Commissioning &Limits");
        commissioningAction->setCheckable(true);
        connect(commissioningAction, &QAction::triggered, this, &SimulatorWindow::ChooseCommissioning);
        connect(armMenu, &QMenu::aboutToShow, this, &SimulatorWindow::UpdateCommissioningItem);
        armMenu->setToolTipsVisible(true);

        armMenu->addSeparator();
        BuildSingleStepMenu(*armMenu->addMenu("Single &Step"));
    }

    void SimulatorWindow::UpdateCommissioningItem()
    {
        const auto& state = presenter.State(Control::commissioning);
        commissioningAction->setChecked(presenter.Commissioning());
        commissioningAction->setEnabled(state.enabled);
        commissioningAction->setToolTip(ui::backend::qt::ToQt(state.reason));
        commissioningAction->setStatusTip(ui::backend::qt::ToQt(state.reason));
    }

    void SimulatorWindow::ChooseCommissioning(bool on)
    {
        if (on)
        {
            presenter.SetCommissioning(true);
            return;
        }

        const auto adapter = QString::fromStdString(host.Snapshot().target.bus);
        QMessageBox confirm{ QMessageBox::Warning, "Use Profile Limits",
            QString{ "Run the real arm on %1 at the profile's full speed and trip limits?\n\nThe arm is rebuilt and must be brought up again." }.arg(adapter),
            QMessageBox::NoButton, this };
        auto* useProfile = confirm.addButton("Use Profile Limits", QMessageBox::AcceptRole);
        confirm.setDefaultButton(confirm.addButton(QMessageBox::Cancel));
        confirm.exec();
        if (confirm.clickedButton() == useProfile)
            presenter.SetCommissioning(false);
        else
            commissioningAction->setChecked(true);
    }

    void SimulatorWindow::BuildSingleStepMenu(QMenu& menu)
    {
        menu.setToolTipsVisible(true);
        for (const auto& item : singleStepItems)
        {
            auto* action = menu.addAction(ui::backend::qt::ToQt(item.label), this, [this, control = item.control]()
                {
                    presenter.SingleStep(control);
                });
            singleStepActions.push_back({ action, item.control });
        }

        connect(&menu, &QMenu::aboutToShow, this, &SimulatorWindow::UpdateSingleStepMenu);
    }

    void SimulatorWindow::UpdateSingleStepMenu()
    {
        for (const auto& [action, control] : singleStepActions)
        {
            const auto& state = presenter.State(control);
            action->setEnabled(state.enabled);
            action->setToolTip(ui::backend::qt::ToQt(state.reason));
            action->setStatusTip(ui::backend::qt::ToQt(state.reason));
        }
    }

    void SimulatorWindow::BuildSimulationMenu()
    {
        simulationMenu = menuBar()->addMenu("&Simulation");
        auto* fault = simulationMenu->addMenu("Inject &Fault");
        faultMenu = fault;
        auto* silence = simulationMenu->addMenu("&Silence Joint");
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            fault->addAction(QString{ "Joint %1" }.arg(joint + 1), this, [this, joint]()
                {
                    presenter.InjectFault(joint);
                });
            silence->addAction(QString{ "Joint %1" }.arg(joint + 1), this, [this, joint]()
                {
                    presenter.SilenceJoint(joint);
                });
        }
    }

    void SimulatorWindow::BuildViewMenu()
    {
        auto* view = menuBar()->addMenu("&View");
        auto* camera = view->addMenu("&Camera");
        const std::array<const char*, 5> presets{ "&Front\t1", "&Side\t2", "&Top\t3", "&Iso\t4", "Frame &Arm\tF" };
        for (int preset = 0; preset != static_cast<int>(presets.size()); ++preset)
            camera->addAction(presets[static_cast<std::size_t>(preset)], this, [this, preset]()
                {
                    sceneState.RequestCameraPreset(preset);
                });
        camera->addSeparator();
        auto* focusJoint = camera->addAction("Focus Selected &Joint\tJ", this, [this]()
            {
                sceneState.RequestFocusSelected();
            });
        connect(camera, &QMenu::aboutToShow, this, [this, focusJoint]()
            {
                focusJoint->setEnabled(!sceneState.SelectionText().isEmpty());
                focusJoint->setToolTip(sceneState.SelectionText().isEmpty() ? "Select a joint in the 3-D view or the joint table first" : QString{});
            });
        auto* orthographic = camera->addAction("&Orthographic\tO");
        orthographic->setCheckable(true);
        orthographic->setChecked(sceneState.Orthographic());
        connect(orthographic, &QAction::toggled, &sceneState, &ArmSceneState::SetOrthographic);
        connect(&sceneState, &ArmSceneState::overlaysChanged, orthographic, [this, orthographic]()
            {
                orthographic->setChecked(sceneState.Orthographic());
            });

        BuildOverlaysMenu(*view->addMenu("&Overlays"));

        auto* theme = view->addMenu("&Theme");
        theme->addAction("&Instrument", this, [this]()
            {
                UseTheme(false);
            });
        theme->addAction("&Light", this, [this]()
            {
                UseTheme(true);
            });

        view->addSeparator();
        for (auto* dock : findChildren<QDockWidget*>())
            view->addAction(dock->toggleViewAction());
        view->addSeparator();
        view->addAction("&Reset Layout", QKeySequence{ "Ctrl+Shift+R" }, this, &SimulatorWindow::ResetLayout);
    }

    void SimulatorWindow::BuildOverlaysMenu(QMenu& menu)
    {
        using Overlay = ArmSceneState::Overlay;
        const std::array<std::pair<Overlay, const char*>, ArmSceneState::overlayCount> overlays{ {
            { Overlay::ghost, "&Commanded Arm (Ghost)" },
            { Overlay::trail, "Tool &Trail" },
            { Overlay::frames, "Tool and Target &Frames" },
            { Overlay::limits, "Joint &Limits" },
            { Overlay::workspace, "&Workspace and Zones" },
            { Overlay::legend, "L&egend" },
        } };
        const std::array<const char*, ArmSceneState::overlayCount> keys{ "ghost", "trail", "frames", "limits", "workspace", "legend" };
        for (std::size_t index = 0; index != overlays.size(); ++index)
        {
            const auto [overlay, label] = overlays[index];
            const auto key = QString{ "overlays/" } + keys[index];
            sceneState.SetShown(overlay, Settings().value(key, true).toBool());
            auto* action = menu.addAction(label);
            action->setCheckable(true);
            action->setChecked(sceneState.Shows(overlay));
            connect(action, &QAction::toggled, this, [this, overlay, key](bool on)
                {
                    sceneState.SetShown(overlay, on);
                    Settings().setValue(key, on);
                });
        }
        menu.addSeparator();
        auto* note = menu.addAction("A stop by the workspace rule always shows its box");
        note->setEnabled(false);
    }

    void SimulatorWindow::ConnectSignals()
    {
        connect(strip, &StatusStripBar::emergencyStopPressed, this, [this]()
            {
                presenter.EmergencyStop();
            });
        connect(strip, &StatusStripBar::targetChosen, this, &SimulatorWindow::ChooseTarget);
        connect(virtualMotorsPanel, &VirtualMotorsPanel::restartRequested, this, [this](int joint)
            {
                presenter.RestartMotor(static_cast<std::size_t>(joint));
            });
        connect(joints, &JointTable::jointSelected, this, [this](int joint)
            {
                presenter.SelectJoint(joint);
                scope->SelectJoint(static_cast<std::size_t>(joint));
            });
        connect(&sceneState, &ArmSceneState::meshFailed, this, &SimulatorWindow::NoteMesh);
        connect(tuningPanel->ExportButton(), &QPushButton::clicked, this, &SimulatorWindow::ExportTuningRun);
        connect(&sceneState, &ArmSceneState::jointPicked, this, [this](int joint)
            {
                if (presenter.SelectJoint(joint))
                    scope->SelectJoint(static_cast<std::size_t>(joint));
            });
        const auto selectElsewhere = [this](std::size_t joint)
        {
            if (presenter.SelectJoint(static_cast<int>(joint)))
                scope->SelectJoint(joint);
        };
        motorController.SetJointSelected(selectElsewhere);
        motor.SetJointSelected(selectElsewhere);
        presenter.SetRearmPrompt([this](std::string_view prompt)
            {
                ConfirmRearm(prompt);
            });
        presenter.SetDisablePrompt([this](std::string_view prompt)
            {
                ConfirmDisable(prompt);
            });
        armControllers.SetDisablePrompt([this](std::string_view prompt)
            {
                ConfirmControllerDisable(prompt);
            });
        connect(armControllersPanel, &ArmControllersPanel::chosen, this, [this](int row)
            {
                armControllers.Choose(row < 0 ? std::nullopt : std::optional<std::size_t>{ static_cast<std::size_t>(row) });
                armControllersPanel->Update(armControllers);
            });
        connect(armControllersPanel, &ArmControllersPanel::actionTriggered, this, [this](int action)
            {
                armControllers.Trigger(static_cast<ArmControllerAction>(action));
            });
        ConnectJog();
        ConnectTeach();
    }

    void SimulatorWindow::ConnectJog()
    {
        connect(jogPanel, &JogPanel::spaceChosen, this, [this](int space)
            {
                jog.SetSpace(static_cast<JogSpace>(space));
                jogPanel->Update(jog);
            });
        connect(jogPanel, &JogPanel::shareChanged, this, [this](int percent)
            {
                jog.SetShare(percent);
                Settings().setValue("jog/share", jog.Share());
                jogPanel->Update(jog);
            });
        connect(jogPanel, &JogPanel::axisChosen, this, [this](int axis)
            {
                jog.ChooseAxis(static_cast<std::size_t>(axis));
                jogPanel->Update(jog);
            });
        connect(jogPanel, &JogPanel::pressed, this, [this](int axis, int direction)
            {
                jog.Press(static_cast<std::size_t>(axis), direction, std::chrono::steady_clock::now());
                jogPanel->Update(jog);
            });
        connect(jogPanel, &JogPanel::released, this, [this]()
            {
                jog.Release();
                jogPanel->Update(jog);
            });
        connect(qApp, &QGuiApplication::applicationStateChanged, this, [this](Qt::ApplicationState state)
            {
                if (state != Qt::ApplicationActive)
                    jog.Release();
            });
        connect(qApp, &QApplication::focusChanged, this, [this](QWidget*, QWidget* now)
            {
                if (now == nullptr || (now != jogPanel && !jogPanel->isAncestorOf(now)))
                    jog.Release();
            });
        jog.SetShare(Settings().value("jog/share", defaultJogShare).toInt());
    }

    void SimulatorWindow::ConnectTeach()
    {
        connect(teachPanel, &TeachPanel::chosen, this, [this](int row)
            {
                teach.Choose(row < 0 ? std::nullopt : std::optional<std::size_t>{ static_cast<std::size_t>(row) });
                teachPanel->Update(teach);
            });
        connect(teachPanel, &TeachPanel::nameEdited, this, [this](const QString& name)
            {
                teach.SetName(name.toStdString());
                teachPanel->Update(teach);
            });
        connect(teachPanel, &TeachPanel::teach, this, [this]()
            {
                teach.Teach();
            });
        connect(teachPanel, &TeachPanel::goTo, this, [this]()
            {
                teach.GoTo();
            });
    }

    void SimulatorWindow::ChooseTarget(int choice)
    {
        auto settings = Settings();
        bool accepted = false;

        if (choice == StatusStripBar::simulatedJoints)
            presenter.SwitchTarget({ arm_host::TargetKind::simulated, "" });
        else if (choice == StatusStripBar::virtualArm)
            ChooseVirtualArm();
        else if (choice == StatusStripBar::virtualBus)
        {
            const auto bus = QInputDialog::getText(this, "Virtual Bus", "Virtual CAN bus:", QLineEdit::Normal, settings.value("virtualBus", "virtual:arm").toString(), &accepted);
            if (accepted && !bus.isEmpty())
            {
                settings.setValue("virtualBus", bus);
                presenter.SwitchTarget({ arm_host::TargetKind::virtualBus, bus.toStdString() });
            }
        }
        else
            ChooseRealArm();
    }

    void SimulatorWindow::ChooseRealArm()
    {
        auto* dialog = new AdapterDialog{ Settings().value("realAdapter").toString(), this };
        connect(dialog, &AdapterDialog::searchRequested, this, [this, dialog]()
            {
                SearchAdapters(*dialog);
            });
        connect(dialog, &QDialog::accepted, this, [this, dialog]()
            {
                ConfirmRealArm(dialog->Adapter());
            });
        SearchAdapters(*dialog);
        dialog->open();
    }

    void SimulatorWindow::SearchAdapters(AdapterDialog& dialog)
    {
        const auto search = dialog.StartSearch();
        const QPointer<AdapterDialog> guard{ &dialog };
        QTimer::singleShot(std::chrono::seconds{ adapterSearchSeconds }, &dialog, [guard, search]()
            {
                if (guard)
                    guard->TimeOut(search);
            });
        host.DetectAdapters(adapterDetector, [guard, search](std::vector<std::string> adapters)
            {
                QMetaObject::invokeMethod(
                    qApp, [guard, search, adapters = std::move(adapters)]()
                    {
                        if (guard)
                            guard->Show(search, ChooseAdapter(adapters, guard->Adapter().toStdString()));
                    },
                    Qt::QueuedConnection);
            });
    }

    void SimulatorWindow::ConfirmRealArm(const QString& adapter)
    {
        if (adapter.isEmpty())
            return;

        QMessageBox confirm{ QMessageBox::Warning, "Connect to the Real Arm", QString{ "Connect to the real arm on %1?\n\nJoints start stopped." }.arg(adapter),
            QMessageBox::NoButton, this };
        auto* connectButton = confirm.addButton("Connect", QMessageBox::AcceptRole);
        confirm.setDefaultButton(confirm.addButton(QMessageBox::Cancel));
        confirm.exec();
        if (confirm.clickedButton() == connectButton)
        {
            Settings().setValue("realAdapter", adapter);
            presenter.SwitchTarget({ arm_host::TargetKind::real, adapter.toStdString() });
        }
    }

    void SimulatorWindow::SetAdapterDetector(std::function<std::vector<std::string>()> detector)
    {
        adapterDetector = std::move(detector);
    }

    void SimulatorWindow::ChooseVirtualArm()
    {
        auto settings = Settings();
        const auto fromEnvironment = qEnvironmentVariable("E_FOC_VIRTUAL_MOTOR", "e-foc-virtual-motor");
        bool accepted = false;
        const auto engine = QInputDialog::getText(this, "Virtual Arm", "e-foc virtual motor binary:", QLineEdit::Normal,
            settings.value("virtualMotorEngine", fromEnvironment).toString(), &accepted);
        if (!accepted || engine.isEmpty())
            return;

        settings.setValue("virtualMotorEngine", engine);
        presenter.SwitchTarget({ arm_host::TargetKind::virtualArm, "virtual:arm", engine.toStdString() });
    }

    void SimulatorWindow::ConfirmRearm(std::string_view prompt)
    {
        QMessageBox confirm{ QMessageBox::Question, "Re-arm", ui::backend::qt::ToQt(prompt), QMessageBox::NoButton, this };
        auto* rearmButton = confirm.addButton("Re-arm", QMessageBox::AcceptRole);
        confirm.setDefaultButton(confirm.addButton(QMessageBox::Cancel));
        confirm.exec();
        if (confirm.clickedButton() == rearmButton)
            presenter.ConfirmRearm();
    }

    void SimulatorWindow::ConfirmDisable(std::string_view prompt)
    {
        QMessageBox confirm{ QMessageBox::Warning, "Disable While Holding", ui::backend::qt::ToQt(prompt), QMessageBox::NoButton, this };
        auto* disableButton = confirm.addButton("Disable", QMessageBox::DestructiveRole);
        confirm.setDefaultButton(confirm.addButton(QMessageBox::Cancel));
        confirm.exec();
        if (confirm.clickedButton() == disableButton)
            presenter.ConfirmDisable();
    }

    void SimulatorWindow::ConfirmControllerDisable(std::string_view prompt)
    {
        QMessageBox confirm{ QMessageBox::Warning, "Disable Arm Controller", ui::backend::qt::ToQt(prompt), QMessageBox::NoButton, this };
        auto* disableButton = confirm.addButton("Disable", QMessageBox::DestructiveRole);
        confirm.setDefaultButton(confirm.addButton(QMessageBox::Cancel));
        confirm.exec();
        if (confirm.clickedButton() == disableButton)
            armControllers.ConfirmDisable();
    }

    void SimulatorWindow::SaveLog()
    {
        const auto path = QFileDialog::getSaveFileName(this, "Save Log", "e-robot-arm-simulator.log", "Log files (*.log *.txt)");
        if (!path.isEmpty() && !logPanel->SaveTo(path))
            shell.ShowAlert("Save Log", "The log could not be written to " + path.toStdString() + ".");
    }

    void SimulatorWindow::ExportTuningRun()
    {
        const auto path = QFileDialog::getSaveFileName(this, "Export Tuning Run", "tuning-run.csv", "CSV files (*.csv)");
        if (path.isEmpty())
            return;
        std::ofstream file{ path.toStdString() };
        tuning.WriteCsv(file);
        if (!file)
            shell.ShowAlert("Export Tuning Run", "The run could not be written to " + path.toStdString() + ".");
    }

    void SimulatorWindow::OpenProfile()
    {
        const auto path = QFileDialog::getOpenFileName(this, "Open Profile", profilePath, "Arm profiles (*.json)");
        if (!path.isEmpty())
            LoadProfileFrom(path);
    }

    void SimulatorWindow::LoadProfileFrom(const QString& path)
    {
        auto read = arm_host::ReadProfileFile(path.toStdString());
        if (const auto* problem = std::get_if<std::string>(&read))
        {
            shell.ShowAlert("Open Profile", "The profile was not loaded.\n\n" + *problem);
            return;
        }

        const auto& profile = std::get<arm::ArmProfile>(read);
        const auto snapshot = host.Snapshot();
        if (snapshot.target.kind == arm_host::TargetKind::real)
        {
            QMessageBox confirm{ QMessageBox::Warning, "Load Profile on the Real Arm",
                QString{ "Load profile \"%1\" onto the real arm on %2?\n\nThe arm is rebuilt and must be brought up again." }
                    .arg(QString::fromUtf8(profile.name.data(), static_cast<qsizetype>(profile.name.size())), ui::backend::qt::ToQt(snapshot.target.bus)),
                QMessageBox::NoButton, this };
            auto* loadButton = confirm.addButton("Load", QMessageBox::AcceptRole);
            confirm.setDefaultButton(confirm.addButton(QMessageBox::Cancel));
            confirm.exec();
            if (confirm.clickedButton() != loadButton)
                return;
        }

        presenter.LoadProfile(profile);
        MeshNames names{};
        for (std::size_t node = 0; node != names.size(); ++node)
            names[node].assign(profile.meshes[node].begin(), profile.meshes[node].end());
        pendingMeshes.emplace(names, QFileInfo{ path }.absolutePath());
        RememberProfile(path);
    }

    void SimulatorWindow::RememberProfile(const QString& path)
    {
        profilePath = path;
        recentProfiles.Add(path.toStdString());
        QStringList stored;
        for (const auto& recent : recentProfiles.Paths())
            stored.append(QString::fromStdString(recent));
        auto settings = Settings();
        settings.setValue("profile", path);
        settings.setValue("recentProfiles", stored);
        BuildRecentMenu();
    }

    void SimulatorWindow::BuildRecentMenu()
    {
        recentMenu->clear();
        for (const auto& entry : recentProfiles.Entries())
        {
            auto* action = recentMenu->addAction(QString::fromStdString(entry.label));
            action->setToolTip(QString::fromStdString(entry.tooltip));
            action->setEnabled(entry.available);
            const auto path = QString::fromStdString(entry.path);
            connect(action, &QAction::triggered, this, [this, path]()
                {
                    LoadProfileFrom(path);
                });
        }
        recentMenu->addSeparator();
        auto* clear = recentMenu->addAction("&Clear Recent", this, [this]()
            {
                recentProfiles.Clear();
                Settings().remove("recentProfiles");
                BuildRecentMenu();
            });
        clear->setEnabled(!recentProfiles.Paths().empty());
    }

    void SimulatorWindow::SaveProfile()
    {
        const auto path = QFileDialog::getSaveFileName(this, "Save Profile", profilePath.isEmpty() ? QString{ "arm-profile.json" } : profilePath, "Arm profiles (*.json)");
        if (!path.isEmpty())
            SaveProfileTo(path);
    }

    void SimulatorWindow::SaveProfileTo(const QString& path)
    {
        const auto snapshot = host.Snapshot();
        arm::ArmProfile profile{ {}, snapshot.config };
        profile.name.assign(snapshot.profileName.substr(0, arm::profileNameSize));
        const QDir target{ QFileInfo{ path }.absolutePath() };
        for (std::size_t node = 0; node != meshNames.size(); ++node)
            if (!meshNames[node].empty())
                profile.meshes[node].assign(target.relativeFilePath(QDir{ meshDirectory }.absoluteFilePath(QString::fromStdString(meshNames[node]))).toStdString().substr(0, arm::meshPathSize));
        const auto problem = arm_host::WriteProfileFile(path.toStdString(), profile);
        if (!problem.empty())
            return shell.ShowAlert("Save Profile", "The profile was not saved.\n\n" + problem);

        RememberProfile(path);
        shell.SetStatus("Profile saved to " + path.toStdString());
    }

    void SimulatorWindow::CaptureDefaultLayout()
    {
        const auto saved = saveState();
        resizeDocks({ findChild<QDockWidget*>("scopeDock") }, { 230 }, Qt::Vertical);
        resizeDocks({ findChild<QDockWidget*>("armDock") }, { 480 }, Qt::Horizontal);
        defaultLayout = saveState();
        if (Settings().contains("layout"))
            restoreState(saved);
    }

    void SimulatorWindow::ResetLayout()
    {
        restoreState(defaultLayout);
        for (auto* dock : findChildren<QDockWidget*>())
            dock->show();
    }

    void SimulatorWindow::UseTheme(bool light)
    {
        const auto& theme = light ? ui::theme::Light() : ui::theme::Instrument();
        ui::backend::qt::ApplyTheme(theme);
        sceneState.ApplyTheme(theme);
        strip->ApplyTheme();
        motorControllerPanel->ApplyTheme();
        motorPanel->ApplyTheme();
        armControllersPanel->ApplyTheme();
        jogPanel->ApplyTheme();
        teachPanel->ApplyTheme();
        tool->ApplyTheme();
        virtualMotorsPanel->ApplyTheme();
        disturbancesPanel->ApplyTheme();
        tuningPanel->ApplyTheme();
        ui::backend::qt::StyleStatusLabel(*banner, ui::theme::StatusLevel::Warning);
        Settings().setValue("lightTheme", light);
    }

    void SimulatorWindow::ShowAbout()
    {
        const auto api = armView->quickWindow() != nullptr ? GraphicsApiName(armView->quickWindow()->rendererInterface()->graphicsApi()) : QString{ "unknown" };
        QMessageBox::about(this, "About e-robot-arm Simulator",
            QString{ "e-robot-arm Simulator %1\n\nQt %2, rendering with %3." }.arg(E_ROBOT_ARM_VERSION, qVersion(), api));
    }

    void SimulatorWindow::RestoreSettings()
    {
        auto settings = Settings();
        std::vector<std::string> stored;
        for (const auto& recent : settings.value("recentProfiles").toStringList())
            stored.push_back(recent.toStdString());
        recentProfiles.Restore(stored);
        BuildRecentMenu();

        if (options.safeStart)
        {
            UseTheme(false);
            issueLog.Write(ui::backend::qt::update::IssueSeverity::warning, "Safe start: saved settings and last profile not restored");
            return;
        }

        UseTheme(settings.value("lightTheme", false).toBool());
        restoreGeometry(settings.value("geometry").toByteArray());
        restoreState(settings.value("layout").toByteArray());
        issueLog.Write(ui::backend::qt::update::IssueSeverity::info, "Saved settings restored");

        const auto profile = settings.value("profile").toString();
        if (!profile.isEmpty() && QFileInfo::exists(profile))
        {
            issueLog.Write(ui::backend::qt::update::IssueSeverity::info, "Last profile: " + QDir::toNativeSeparators(profile));
            LoadProfileFrom(profile);
        }
    }

    void SimulatorWindow::closeEvent(QCloseEvent* event)
    {
        issueLog.Write(ui::backend::qt::update::IssueSeverity::info, "Window closing; saving settings");
        auto settings = Settings();
        settings.setValue("geometry", saveGeometry());
        settings.setValue("layout", saveState());
        QMainWindow::closeEvent(event);
    }
}
