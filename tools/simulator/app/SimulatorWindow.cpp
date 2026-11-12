#include "tools/simulator/app/SimulatorWindow.hpp"
#include "ui/backend/qt/QtConversions.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include <QAction>
#include <QCloseEvent>
#include <QDockWidget>
#include <QFileDialog>
#include <QGroupBox>
#include <QInputDialog>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QQmlEngine>
#include <QQuickItem>
#include <QQuickWidget>
#include <QScrollArea>
#include <QSettings>
#include <QToolBar>
#include <QVBoxLayout>
#include <QtGlobal>

namespace simulator
{
    namespace
    {
        constexpr ui::shell::ShellSpec shellSpec{ "e-robot-arm Simulator", { 1600.0f, 900.0f }, 420.0f, {}, "Ready" };

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

    SimulatorWindow::SimulatorWindow(arm_host::ArmHost& host, const arm::JointLimits& limits, infra::TimePoint start)
        : host{ host }
        , shell{ *this, shellSpec }
        , strip{ new StatusStripBar{ this } }
        , banner{ new QLabel{ this } }
        , armView{ new QQuickWidget{ this } }
        , joints{ new JointTable{ this } }
        , operate{ new FormPanel{ this } }
        , move{ new FormPanel{ this } }
        , scope{ new ScopePanel{ this } }
        , presenter{ host, shell, operate->Form(), move->Form(), limits, start }
        , logPanel{ new EventLogPanel{ presenter.Log(), this } }
    {
        setMinimumSize(1280, 720);
        ArmSceneState::Use(sceneState);
        armView->setResizeMode(QQuickWidget::SizeRootObjectToView);
        armView->setAccessibleName("3-D arm view");
        armView->setFocusPolicy(Qt::StrongFocus);
        armView->engine()->addImportPath(":/");
        armView->setSource(QUrl{ "qrc:/Simulator/qml/ArmScene.qml" });
        EnableAmbientOcclusion();
        shell.SetContent(armView);

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
        auto* environment = root != nullptr ? root->findChild<QObject*>("environment") : nullptr;
        if (environment != nullptr && environment->metaObject()->indexOfProperty("aoEnabled") >= 0)
            environment->setProperty("aoEnabled", true);
    }

    QQuickWidget& SimulatorWindow::ArmView()
    {
        return *armView;
    }

    SimulatorPresenter& SimulatorWindow::Presenter()
    {
        return presenter;
    }

    void SimulatorWindow::Frame()
    {
        const auto snapshot = host.Snapshot();
        const auto events = host.TakeEvents();
        const auto trace = host.TakeTrace();

        presenter.Refresh(snapshot, events);
        const auto& status = presenter.Status();
        strip->Update(status, presenter.State(Control::emergencyStop), presenter.State(Control::target), snapshot.target.kind);
        UpdateBanner(status);

        joints->Update(presenter.Rows());
        joints->Select(presenter.Scene().Selected());
        operate->SetHint(presenter.OperateHint());
        move->SetHint(presenter.MoveHint());

        scope->Add(trace);
        if (std::ranges::any_of(events, [](const arm_host::ArmEvent& event)
                {
                    return event.kind == arm_host::ArmEventKind::tripped;
                }))
            scope->TriggerOnTrip();
        scope->Repaint();

        logPanel->AppendNewest(events.size());
        sceneState.Update(presenter.Scene());
        simulationMenu->setEnabled(snapshot.targetReady && snapshot.target.kind == arm_host::TargetKind::simulated);
    }

    void SimulatorWindow::UpdateBanner(const StatusStrip& status)
    {
        banner->parentWidget()->setVisible(status.realArm);
        banner->setText(ui::backend::qt::ToQt("  " + status.target + " — commands move real hardware"));
        setWindowTitle(ui::backend::qt::ToQt("e-robot-arm Simulator — " + status.target));
    }

    void SimulatorWindow::BuildDocks()
    {
        setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowTabbedDocks);
        auto* column = new QWidget{ this };
        auto* layout = new QVBoxLayout{ column };
        layout->setContentsMargins(8, 8, 8, 8);
        layout->setSpacing(16);
        layout->addWidget(Group("Joints", joints));
        layout->addWidget(Group("Operate", operate));
        layout->addWidget(Group("Move", move));
        layout->addStretch();

        auto* scroll = new QScrollArea{ this };
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setWidget(column);
        scroll->setMinimumWidth(480);
        Dock("Arm", "armDock", scroll, Qt::RightDockWidgetArea);
        Dock("Scope", "scopeDock", scope, Qt::BottomDockWidgetArea);
        Dock("Event Log", "logDock", logPanel, Qt::BottomDockWidgetArea);
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
        file->addAction("&Save Log…", QKeySequence::Save, this, &SimulatorWindow::SaveLog);
        file->addSeparator();
        file->addAction("&Quit", QKeySequence::Quit, this, &QWidget::close);

        BuildArmMenu();
        BuildSimulationMenu();
        BuildViewMenu();

        auto* help = menuBar()->addMenu("&Help");
        help->addAction("&About", this, &SimulatorWindow::ShowAbout);
    }

    void SimulatorWindow::BuildArmMenu()
    {
        auto* armMenu = menuBar()->addMenu("&Arm");
        auto* emergencyStop = armMenu->addAction("&Emergency Stop", this, [this]()
            {
                presenter.EmergencyStop();
            });
        emergencyStop->setShortcut(Qt::Key_Escape);
        emergencyStop->setShortcutContext(Qt::ApplicationShortcut);
        addAction(emergencyStop);

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
    }

    void SimulatorWindow::BuildSimulationMenu()
    {
        simulationMenu = menuBar()->addMenu("&Simulation");
        auto* fault = simulationMenu->addMenu("Inject &Fault");
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

    void SimulatorWindow::ConnectSignals()
    {
        connect(strip, &StatusStripBar::emergencyStopPressed, this, [this]()
            {
                presenter.EmergencyStop();
            });
        connect(strip, &StatusStripBar::targetChosen, this, &SimulatorWindow::ChooseTarget);
        connect(joints, &JointTable::jointSelected, this, [this](int joint)
            {
                presenter.SelectJoint(joint);
                scope->SelectJoint(static_cast<std::size_t>(joint));
            });
        connect(&sceneState, &ArmSceneState::jointPicked, this, [this](int joint)
            {
                if (presenter.SelectJoint(joint))
                    scope->SelectJoint(static_cast<std::size_t>(joint));
            });
        presenter.SetRearmPrompt([this](std::string_view prompt)
            {
                ConfirmRearm(prompt);
            });
    }

    void SimulatorWindow::ChooseTarget(int choice)
    {
        auto settings = Settings();
        bool accepted = false;

        if (choice == StatusStripBar::simulatedJoints)
            host.SwitchTarget({ arm_host::TargetKind::simulated, "" });
        else if (choice == StatusStripBar::virtualBus)
        {
            const auto bus = QInputDialog::getText(this, "Virtual Bus", "Virtual CAN bus:", QLineEdit::Normal, settings.value("virtualBus", "virtual:arm").toString(), &accepted);
            if (accepted && !bus.isEmpty())
            {
                settings.setValue("virtualBus", bus);
                host.SwitchTarget({ arm_host::TargetKind::virtualBus, bus.toStdString() });
            }
        }
        else
        {
            const auto adapter = QInputDialog::getText(this, "Real Arm", "CAN adapter (socketcan:<interface> or slcan:<port>):", QLineEdit::Normal,
                settings.value("realAdapter", "socketcan:can0").toString(), &accepted);
            if (!accepted || adapter.isEmpty())
                return;

            QMessageBox confirm{ QMessageBox::Warning, "Connect to the Real Arm", QString{ "Connect to the real arm on %1?\n\nJoints start stopped." }.arg(adapter),
                QMessageBox::NoButton, this };
            auto* connectButton = confirm.addButton("Connect", QMessageBox::AcceptRole);
            confirm.setDefaultButton(confirm.addButton(QMessageBox::Cancel));
            confirm.exec();
            if (confirm.clickedButton() == connectButton)
            {
                settings.setValue("realAdapter", adapter);
                host.SwitchTarget({ arm_host::TargetKind::real, adapter.toStdString() });
            }
        }
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

    void SimulatorWindow::SaveLog()
    {
        const auto path = QFileDialog::getSaveFileName(this, "Save Log", "e-robot-arm-simulator.log", "Log files (*.log *.txt)");
        if (!path.isEmpty() && !logPanel->SaveTo(path))
            shell.ShowAlert("Save Log", "The log could not be written to " + path.toStdString() + ".");
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
        ui::backend::qt::StyleStatusLabel(*banner, ui::theme::StatusLevel::Warning);
        Settings().setValue("lightTheme", light);
    }

    void SimulatorWindow::ShowAbout()
    {
        const auto api = armView->quickWindow() != nullptr ? GraphicsApiName(armView->quickWindow()->rendererInterface()->graphicsApi()) : QString{ "unknown" };
        QMessageBox::about(this, "About e-robot-arm Simulator",
            QString{ "e-robot-arm Simulator\n\nQt %1, rendering with %2." }.arg(qVersion(), api));
    }

    void SimulatorWindow::RestoreSettings()
    {
        auto settings = Settings();
        UseTheme(settings.value("lightTheme", false).toBool());
        restoreGeometry(settings.value("geometry").toByteArray());
        restoreState(settings.value("layout").toByteArray());
    }

    void SimulatorWindow::closeEvent(QCloseEvent* event)
    {
        auto settings = Settings();
        settings.setValue("geometry", saveGeometry());
        settings.setValue("layout", saveState());
        QMainWindow::closeEvent(event);
    }
}
