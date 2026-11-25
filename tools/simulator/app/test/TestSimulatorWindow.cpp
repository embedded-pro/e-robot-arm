#include "tools/arm_host/AdapterTargetLink.hpp"
#include "tools/arm_host/DefaultArmConfig.hpp"
#include "tools/arm_host/ProfileFile.hpp"
#include "tools/simulator/app/AdapterDialog.hpp"
#include "tools/simulator/app/ArmSceneState.hpp"
#include "tools/simulator/app/Diagnostics.hpp"
#include "tools/simulator/app/SimulatorWindow.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include "gmock/gmock.h"
#ifdef E_ROBOT_ARM_VIRTUAL_ARM
#include "tools/arm_host/VirtualArmTargetLink.hpp"
#endif
#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPointer>
#include <QPushButton>
#include <QQuickItem>
#include <QQuickWidget>
#include <QQuickWindow>
#include <QSettings>
#include <QSurfaceFormat>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTimer>
#include <QVariantMap>
#include <QVector3D>
#include <QtQml/QQmlExtensionPlugin>
#include <algorithm>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

Q_IMPORT_QML_PLUGIN(SimulatorPlugin)
Q_IMPORT_QML_PLUGIN(UiQuick3DPlugin)

namespace
{
    bool Eventually(const std::function<bool()>& condition, int timeoutMilliseconds = 5000)
    {
        QElapsedTimer timer;
        timer.start();
        while (timer.elapsed() < timeoutMilliseconds)
        {
            QApplication::processEvents(QEventLoop::AllEvents, 10);
            if (condition())
                return true;
        }
        return condition();
    }

    bool Logged(const simulator::SimulatorPresenter& presenter, std::string_view message)
    {
        return std::ranges::any_of(presenter.Log().Lines(), [message](const simulator::LogLine& line)
            {
                return line.message.find(message) != std::string::npos;
            });
    }

    std::string LogText(const simulator::SimulatorPresenter& presenter)
    {
        std::string text;
        for (const auto& line : presenter.Log().Lines())
            text += std::to_string(line.seconds) + " " + line.message + "\n";
        return text;
    }

    bool HasDetail(const QImage& image)
    {
        const auto reference = image.pixel(0, 0);
        int differing = 0;
        for (int y = 0; y < image.height(); y += 8)
            for (int x = 0; x < image.width(); x += 8)
                differing += image.pixel(x, y) != reference ? 1 : 0;
        return differing > 100;
    }

    QLabel* Labelled(QWidget& parent, const QString& accessibleName)
    {
        for (auto* label : parent.findChildren<QLabel*>())
            if (label->accessibleName() == accessibleName)
                return label;
        return nullptr;
    }

    QPushButton* Button(QWidget& parent, const QString& text)
    {
        for (auto* button : parent.findChildren<QPushButton*>())
            if (button->text() == text)
                return button;
        return nullptr;
    }

    QAction* MenuAction(QWidget& window, const QString& text)
    {
        for (auto* action : window.findChildren<QAction*>())
            if (action->text().remove('&').section('\t', 0, 0) == text)
                return action;
        return nullptr;
    }

    QVector3D Corner(const QVariantMap& bounds, int corner)
    {
        const auto min = bounds["min"].value<QVector3D>();
        const auto max = bounds["max"].value<QVector3D>();
        return { (corner & 1) != 0 ? max.x() : min.x(), (corner & 2) != 0 ? max.y() : min.y(), (corner & 4) != 0 ? max.z() : min.z() };
    }

    bool FramesAllCorners(QQuickItem& studio, const QVariantMap& bounds)
    {
        auto* view = studio.property("view").value<QObject*>();
        for (int corner = 0; corner != 8; ++corner)
        {
            QVector3D screen;
            QMetaObject::invokeMethod(view, "mapFrom3DScene", Q_RETURN_ARG(QVector3D, screen), Q_ARG(QVector3D, Corner(bounds, corner)));
            if (screen.x() < 0.0f || screen.y() < 0.0f || screen.x() > studio.width() || screen.y() > studio.height())
                return false;
        }
        return true;
    }

    ui::backend::qt::update::IssueLog::Config LogConfig(const QTemporaryDir& directory)
    {
        auto config = simulator::IssueLogConfig();
        config.directory = directory.path();
        return config;
    }

    bool IssueLogged(const ui::backend::qt::update::IssueLog& log, const QString& message)
    {
        return std::ranges::any_of(log.RecentLines(200), [&message](const QString& line)
            {
                return line.contains(message);
            });
    }

    arm_host::TargetOpener Opener()
    {
        auto opener = arm_host::HostTargetOpener(arm_host::simulatedTimeConstant, 1000000);
#ifdef E_ROBOT_ARM_VIRTUAL_ARM
        opener = arm_host::VirtualArmTargetOpener(std::move(opener), 1000000);
#endif
        return opener;
    }

    class SimulatorWindowTest
        : public testing::Test
    {
    public:
        SimulatorWindowTest()
        {
            window.resize(1600, 900);
            window.show();
        }

        void BringUpAndMove(const arm::JointPositions& target)
        {
            host.Post(arm_host::BringUp{});
            ASSERT_TRUE(Eventually([this]()
                {
                    return window.Presenter().State(simulator::Control::enable).enabled;
                }));
            host.Post(arm_host::Enable{});
            ASSERT_TRUE(Eventually([this]()
                {
                    return window.Presenter().State(simulator::Control::move).enabled;
                }))
                << window.Presenter().MoveHint() << "\n"
                << LogText(window.Presenter());
            host.Post(arm_host::MoveTo{ target });
            ASSERT_TRUE(Eventually([this]()
                {
                    return Logged(window.Presenter(), "Move: accepted");
                }))
                << LogText(window.Presenter());
        }

        arm::ArmProfile profile{ arm_host::DefaultArmProfile(arm_host::defaultNodes) };
        infra::TimePoint start{ std::chrono::system_clock::now() };
        arm_host::ArmHost host{ profile, Opener(), {} };
        QTemporaryDir logDirectory;
        ui::backend::qt::update::IssueLog issueLog{ LogConfig(logDirectory) };
        simulator::SimulatorWindow window{ host, profile.config.positionLimits, start, issueLog };
    };

    class SafeStartTest
        : public testing::Test
    {
    public:
        SafeStartTest()
        {
            window.show();
        }

        ~SafeStartTest() override
        {
            QSettings{ "embedded-pro", "e-robot-arm Simulator" }.remove("profile");
        }

        static bool StoreLastProfile()
        {
            QSettings{ "embedded-pro", "e-robot-arm Simulator" }.setValue("profile", SAMPLE_ARM_PROFILE);
            return true;
        }

        bool stored{ StoreLastProfile() };
        arm::ArmProfile profile{ arm_host::DefaultArmProfile(arm_host::defaultNodes) };
        arm_host::ArmHost host{ profile, Opener(), {} };
        QTemporaryDir logDirectory;
        ui::backend::qt::update::IssueLog issueLog{ LogConfig(logDirectory) };
        simulator::SimulatorWindow window{ host, profile.config.positionLimits, std::chrono::system_clock::now(), issueLog, simulator::StartOptions{ true } };
    };
}

TEST_F(SimulatorWindowTest, writes_the_start_up_milestones_to_the_issue_log)
{
    EXPECT_TRUE(Eventually([this]()
        {
            return IssueLogged(issueLog, "First frame rendered with ");
        }))
        << issueLog.RecentLines(50).join('\n').toStdString();
    EXPECT_TRUE(IssueLogged(issueLog, "Arm scene: ready"));
    EXPECT_TRUE(IssueLogged(issueLog, "Saved settings restored"));
}

TEST_F(SimulatorWindowTest, help_opens_the_log_folder)
{
    EXPECT_NE(nullptr, MenuAction(window, "Open Log Folder"));
}

TEST_F(SafeStartTest, skips_the_saved_settings_and_the_last_profile)
{
    QApplication::processEvents();

    EXPECT_TRUE(IssueLogged(issueLog, "Safe start: saved settings and last profile not restored"));
    EXPECT_FALSE(IssueLogged(issueLog, "Last profile:"));
}

TEST_F(SimulatorWindowTest, loads_the_lit_scene_and_shows_a_move_with_its_ghost)
{
    ASSERT_EQ(QQuickWidget::Ready, window.ArmView().status());
    ASSERT_NE(nullptr, window.ArmView().rootObject());
    auto* warning = window.ArmView().rootObject()->findChild<QQuickItem*>("singularityWarning");
    ASSERT_NE(nullptr, warning);
    auto* position = Labelled(window, "Tool Position");
    ASSERT_NE(nullptr, position);
    EXPECT_TRUE(Eventually([warning, position]()
        {
            return warning->isVisible() && position->text() == "x 0.020  y 0.000  z 0.710 m";
        }));

    const arm::JointPositions target{ foc::Radians{ 0.7f }, foc::Radians{ -0.5f }, foc::Radians{ 0.6f }, foc::Radians{ 0.4f }, foc::Radians{ 0.8f },
        foc::Radians{ 0.0f } };
    BringUpAndMove(target);

    EXPECT_TRUE(Eventually([this]()
        {
            return window.Presenter().Scene().GhostVisible() && window.Presenter().Scene().MeasuredDegrees()[0] > 20.0f;
        }));

    const auto frame = window.ArmView().grabFramebuffer();
    ASSERT_FALSE(frame.isNull());
    EXPECT_TRUE(HasDetail(frame));

    if (const auto* path = std::getenv("SIMULATOR_FRAME_CAPTURE"))
    {
        EXPECT_TRUE(frame.save(path));
        window.grab().save(QString{ path }.replace(".png", "-window.png"));
    }

    EXPECT_TRUE(Eventually([this]()
        {
            return Logged(window.Presenter(), "Move done");
        },
        10000));
    EXPECT_FALSE(warning->isVisible());
    EXPECT_NE("—", position->text());
    auto* trail = window.ArmView().rootObject()->findChild<QObject*>("toolTrail");
    auto* triad = window.ArmView().rootObject()->findChild<QObject*>("toolTriad");
    ASSERT_NE(nullptr, trail);
    ASSERT_NE(nullptr, triad);
    EXPECT_NE(nullptr, window.ArmView().rootObject()->findChild<QObject*>("workspaceBox"));
    EXPECT_TRUE(trail->property("visible").toBool());
    EXPECT_TRUE(triad->property("visible").toBool());
    EXPECT_GT(window.Presenter().Scene().ToolTrail().size(), 2u);

    auto* arc = window.ArmView().rootObject()->findChild<QObject*>("limitArc");
    ASSERT_NE(nullptr, arc);
    EXPECT_FALSE(arc->property("visible").toBool());
    window.Presenter().SelectJoint(1);
    EXPECT_TRUE(Eventually([arc]()
        {
            return arc->property("visible").toBool();
        }));
    EXPECT_NE(nullptr, window.ArmView().rootObject()->findChild<QObject*>("targetMarker"));
    if (const auto* capture = std::getenv("SIMULATOR_FRAME_CAPTURE"))
        window.ArmView().grabFramebuffer().save(QString{ capture }.replace(".png", "-limit-arc.png"));
}

TEST_F(SimulatorWindowTest, an_injected_fault_trips_the_arm_and_names_its_source)
{
    BringUpAndMove(arm::JointPositions{});
    window.Presenter().InjectFault(2);

    EXPECT_TRUE(Eventually([this]()
        {
            return window.Presenter().Status().stop.find("Joint 3") != std::string::npos;
        }));
    EXPECT_EQ(ui::theme::StatusLevel::Fault, window.Presenter().Status().stopLevel);
    EXPECT_FALSE(window.Presenter().State(simulator::Control::move).enabled);
    EXPECT_TRUE(window.Presenter().State(simulator::Control::emergencyStop).enabled);
}

TEST_F(SimulatorWindowTest, the_motor_controller_reads_the_selected_joint_then_applies_and_verifies_it)
{
    host.Post(arm_host::BringUp{});
    ASSERT_TRUE(Eventually([this]()
        {
            return window.Presenter().State(simulator::Control::enable).enabled;
        }));

    auto* dock = window.findChild<QDockWidget*>("motorControllerDock");
    ASSERT_NE(nullptr, dock);
    dock->raise();
    window.Presenter().SelectJoint(2);
    ASSERT_TRUE(Eventually([this]()
        {
            return window.MotorController().Joint() == 2 && window.MotorController().State() == simulator::ConfigurationState::read;
        }));
    auto* apply = Button(*dock, "Apply");
    auto* verify = Button(*dock, "Verify");
    ASSERT_NE(nullptr, apply);
    ASSERT_NE(nullptr, verify);
    EXPECT_FALSE(verify->isEnabled());

    apply->click();
    ASSERT_TRUE(Eventually([this]()
        {
            return window.MotorController().State() == simulator::ConfigurationState::applied;
        }));

    ASSERT_TRUE(Eventually([verify]()
        {
            return verify->isEnabled();
        }));
    verify->click();
    EXPECT_TRUE(Eventually([this]()
        {
            return window.MotorController().State() == simulator::ConfigurationState::verified;
        }));
    EXPECT_EQ(ui::theme::StatusLevel::Ok, window.MotorController().StateLevel());

    if (const auto* path = std::getenv("SIMULATOR_FRAME_CAPTURE"))
    {
        dock->raise();
        QApplication::processEvents();
        window.grab().save(QString{ path }.replace(".png", "-motor-controller.png"));
    }
}

TEST_F(SimulatorWindowTest, the_arm_controllers_dock_selects_a_controller_from_its_list)
{
    auto* dock = window.findChild<QDockWidget*>("armControllersDock");
    ASSERT_NE(nullptr, dock);
    dock->raise();
    auto* table = dock->findChild<QTableWidget*>("armControllersTable");
    ASSERT_NE(nullptr, table);
    ASSERT_TRUE(Eventually([table]()
        {
            return table->rowCount() >= 2;
        }));
    auto* select = Button(*dock, "Select");
    ASSERT_NE(nullptr, select);
    EXPECT_FALSE(select->isEnabled());

    table->selectRow(1);
    ASSERT_TRUE(Eventually([select]()
        {
            return select->isEnabled();
        }));
    select->click();

    EXPECT_TRUE(Eventually([this]()
        {
            const auto controllers = host.Snapshot().armControllers;
            return controllers.size() >= 2 && controllers[1].active;
        }))
        << LogText(window.Presenter());
}

TEST_F(SimulatorWindowTest, the_jog_dock_moves_a_joint_while_its_button_is_held_and_stops_it_on_release)
{
    auto* dock = window.findChild<QDockWidget*>("jogDock");
    ASSERT_NE(nullptr, dock);
    dock->raise();
    auto* plus = Button(*dock, "+");
    ASSERT_NE(nullptr, plus);
    EXPECT_FALSE(plus->isEnabled());

    host.Post(arm_host::BringUp{});
    ASSERT_TRUE(Eventually([this]()
        {
            return window.Presenter().State(simulator::Control::enable).enabled;
        }));
    host.Post(arm_host::Enable{});
    ASSERT_TRUE(Eventually([plus]()
        {
            return plus->isEnabled();
        }))
        << LogText(window.Presenter());

    emit plus->pressed();
    EXPECT_TRUE(Eventually([this]()
        {
            return host.Snapshot().readings[0].position.Value() > 0.05f;
        }))
        << LogText(window.Presenter());
    EXPECT_TRUE(host.Snapshot().moving);

    emit plus->released();
    EXPECT_TRUE(Eventually([this]()
        {
            return !host.Snapshot().moving;
        }))
        << LogText(window.Presenter());
    const auto stopped = host.Snapshot().readings[0].position.Value();
    Eventually([]()
        {
            return false;
        },
        300);
    EXPECT_NEAR(stopped, host.Snapshot().readings[0].position.Value(), 0.01f);
}

TEST_F(SimulatorWindowTest, the_teach_points_dock_teaches_a_pose_and_goes_back_to_it)
{
    auto* dock = window.findChild<QDockWidget*>("teachPointsDock");
    ASSERT_NE(nullptr, dock);
    dock->raise();
    auto* name = dock->findChild<QLineEdit*>();
    ASSERT_NE(nullptr, name);
    auto* teachButton = Button(*dock, "Teach");
    auto* goTo = Button(*dock, "Go To");
    ASSERT_NE(nullptr, teachButton);
    ASSERT_NE(nullptr, goTo);

    name->setText("home");
    emit name->textEdited("home");
    ASSERT_TRUE(Eventually([teachButton]()
        {
            return teachButton->isEnabled();
        }))
        << teachButton->toolTip().toStdString();
    teachButton->click();
    auto* table = dock->findChild<QTableWidget*>("teachPointsTable");
    ASSERT_TRUE(Eventually([table]()
        {
            return table->rowCount() == 1;
        }))
        << LogText(window.Presenter());

    arm::JointPositions target{};
    target[0] = foc::Radians{ 0.3f };
    BringUpAndMove(target);
    ASSERT_TRUE(Eventually([this]()
        {
            return Logged(window.Presenter(), "Move done");
        }));

    table->selectRow(0);
    ASSERT_TRUE(Eventually([goTo]()
        {
            return goTo->isEnabled();
        }))
        << goTo->toolTip().toStdString();
    goTo->click();
    EXPECT_TRUE(Eventually([this]()
        {
            return std::abs(host.Snapshot().readings[0].position.Value()) < 0.01f && !host.Snapshot().moving;
        }))
        << LogText(window.Presenter());
}

TEST_F(SimulatorWindowTest, the_motor_panel_applies_a_preset_and_verifies_it)
{
    host.Post(arm_host::BringUp{});
    ASSERT_TRUE(Eventually([this]()
        {
            return window.Presenter().State(simulator::Control::enable).enabled;
        }));

    auto* dock = window.findChild<QDockWidget*>("motorDock");
    ASSERT_NE(nullptr, dock);
    dock->raise();
    window.Presenter().SelectJoint(4);
    ASSERT_TRUE(Eventually([this]()
        {
            return window.Motor().Joint() == 4 && window.Motor().State() == simulator::ConfigurationState::read;
        }));
    EXPECT_EQ(0u, window.Motor().Preset());

    QComboBox* preset = nullptr;
    for (auto* combo : dock->findChildren<QComboBox*>())
        if (combo->findText("Anaheim BLY172S-24V-4000") >= 0)
            preset = combo;
    ASSERT_NE(nullptr, preset);
    preset->setCurrentIndex(preset->findText("Anaheim BLY172S-24V-4000"));
    EXPECT_EQ(1u, window.Motor().Preset());

    Button(*dock, "Apply")->click();
    ASSERT_TRUE(Eventually([this]()
        {
            return window.Motor().State() == simulator::ConfigurationState::applied;
        }));
    auto* verify = Button(*dock, "Verify");
    ASSERT_TRUE(Eventually([verify]()
        {
            return verify->isEnabled();
        }));
    verify->click();
    EXPECT_TRUE(Eventually([this]()
        {
            return window.Motor().State() == simulator::ConfigurationState::verified;
        }))
        << window.Motor().Hint();

    if (const auto* capture = std::getenv("SIMULATOR_FRAME_CAPTURE"))
    {
        dock->raise();
        QApplication::processEvents();
        window.grab().save(QString{ capture }.replace(".png", "-motor.png"));
    }
}

TEST_F(SimulatorWindowTest, a_loaded_profile_rebuilds_the_arm_and_bring_up_configures_its_motor_controllers)
{
    auto profile = arm_host::DefaultArmProfile({ 41, 42, 43, 44, 45, 46 });
    profile.name = "window test arm";
    arm::ControllerSettings settings{};
    settings.position = can::FocPositionAlgorithm::lqr;
    profile.config.controllers[1] = settings;
    const auto path = std::filesystem::temp_directory_path() / "e-robot-arm-window-test.json";
    ASSERT_TRUE(arm_host::WriteProfileFile(path, profile).empty());

    window.LoadProfileFrom(QString::fromStdString(path.string()));
    ASSERT_TRUE(Eventually([this]()
        {
            return host.Snapshot().profileName == "window test arm";
        }));
    auto* table = window.findChild<QTableWidget*>("profileTable");
    ASSERT_NE(nullptr, table);
    EXPECT_TRUE(Eventually([table]()
        {
            return table->item(1, 1) != nullptr && table->item(1, 1)->text() == "42" && table->item(1, 9)->text() == "PID / PID / LQR";
        }));
    EXPECT_TRUE(Eventually([this]()
        {
            return window.windowTitle().contains("window test arm");
        }))
        << window.windowTitle().toStdString();
    const auto actions = window.findChildren<QAction*>();
    const auto recent = std::ranges::find_if(actions, [](const QAction* action)
        {
            return action->text() == "e-robot-arm-window-test.json";
        });
    ASSERT_NE(actions.end(), recent);
    EXPECT_TRUE((*recent)->isEnabled());

    host.Post(arm_host::BringUp{});
    EXPECT_TRUE(Eventually([this]()
        {
            return window.Presenter().State(simulator::Control::enable).enabled;
        }))
        << LogText(window.Presenter());
    EXPECT_TRUE(Logged(window.Presenter(), "configuring motor controllers"));

    if (const auto* capture = std::getenv("SIMULATOR_FRAME_CAPTURE"))
    {
        window.findChild<QDockWidget*>("profileDock")->raise();
        QApplication::processEvents();
        window.grab().save(QString{ capture }.replace(".png", "-profile.png"));
    }

    std::filesystem::remove(path);
    QSettings{ "embedded-pro", "e-robot-arm Simulator" }.remove("profile");
    QSettings{ "embedded-pro", "e-robot-arm Simulator" }.remove("recentProfiles");
}

TEST_F(SimulatorWindowTest, a_profile_draws_its_link_meshes_and_names_a_mesh_it_cannot_load)
{
    const auto directory = std::filesystem::temp_directory_path() / "e-robot-arm-mesh-test";
    std::filesystem::create_directories(directory / "saved");
    std::filesystem::copy_file(MESH_TEST_DATA "/box.gltf", directory / "box.gltf", std::filesystem::copy_options::overwrite_existing);
    auto profile = arm_host::DefaultArmProfile(arm_host::defaultNodes);
    profile.name = "meshed arm";
    profile.meshes[2] = "box.gltf";
    profile.meshes[3] = "missing.glb";
    ASSERT_TRUE(arm_host::WriteProfileFile(directory / "arm.json", profile).empty());

    window.LoadProfileFrom(QString::fromStdString((directory / "arm.json").string()));
    auto* root = window.ArmView().rootObject();
    EXPECT_TRUE(Eventually([this, root]()
        {
            auto* mesh = root->findChild<QObject*>("mesh2");
            return mesh != nullptr && mesh->property("visible").toBool() && Logged(window.Presenter(), "Link 3 mesh not loaded: cannot open");
        }))
        << LogText(window.Presenter());
    EXPECT_FALSE(root->findChild<QObject*>("mesh3")->property("visible").toBool());
    if (const auto* capture = std::getenv("SIMULATOR_FRAME_CAPTURE"))
        window.ArmView().grabFramebuffer().save(QString{ capture }.replace(".png", "-mesh.png"));

    window.SaveProfileTo(QString::fromStdString((directory / "saved" / "copy.json").string()));
    const auto saved = arm_host::ReadProfileFile(directory / "saved" / "copy.json");
    ASSERT_TRUE(std::holds_alternative<arm::ArmProfile>(saved));
    const auto& meshes = std::get<arm::ArmProfile>(saved).meshes;
    EXPECT_EQ("../box.gltf", std::string(meshes[2].begin(), meshes[2].end()));

    std::filesystem::remove_all(directory);
    QSettings{ "embedded-pro", "e-robot-arm Simulator" }.remove("profile");
}

TEST_F(SimulatorWindowTest, the_sample_meshed_arm_draws_every_link_from_its_mesh)
{
    window.LoadProfileFrom(SAMPLE_ARM_PROFILE);
    auto* root = window.ArmView().rootObject();
    EXPECT_TRUE(Eventually([root]()
        {
            for (int node = 0; node != 7; ++node)
            {
                auto* mesh = root->findChild<QObject*>(QString{ "mesh%1" }.arg(node));
                if (mesh == nullptr || !mesh->property("visible").toBool())
                    return false;
            }
            return true;
        }))
        << LogText(window.Presenter());
    EXPECT_FALSE(Logged(window.Presenter(), "mesh not loaded")) << LogText(window.Presenter());
    if (const auto* capture = std::getenv("SIMULATOR_FRAME_CAPTURE"))
        window.ArmView().grabFramebuffer().save(QString{ capture }.replace(".png", "-sample.png"));

    QSettings{ "embedded-pro", "e-robot-arm Simulator" }.remove("profile");
    QSettings{ "embedded-pro", "e-robot-arm Simulator" }.remove("recentProfiles");
}

TEST_F(SimulatorWindowTest, frame_arm_fits_the_posed_arm_and_the_overlays_follow_the_view_menu)
{
    auto* root = window.ArmView().rootObject();
    auto* studio = root->findChild<QQuickItem*>("studio");
    ASSERT_NE(nullptr, studio);
    auto* state = simulator::ArmSceneState::Current();
    ASSERT_NE(nullptr, state);
    const auto frameAndSettle = [this, state]()
    {
        state->RequestCameraPreset(4);
        QElapsedTimer settle;
        settle.start();
        while (settle.elapsed() < 600)
            QApplication::processEvents(QEventLoop::AllEvents, 10);
    };
    const auto capture = [this](const char* suffix)
    {
        if (const auto* path = std::getenv("SIMULATOR_FRAME_CAPTURE"))
            window.ArmView().grabFramebuffer().save(QString{ path }.replace(".png", suffix));
    };

    frameAndSettle();
    EXPECT_TRUE(FramesAllCorners(*studio, state->FrameBounds().toMap()));
    capture("-folded.png");

    BringUpAndMove(arm::JointPositions{ foc::Radians{ 0.6f }, foc::Radians{ 1.2f }, foc::Radians{ -0.9f }, foc::Radians{ 0.0f }, foc::Radians{ 0.5f },
        foc::Radians{ 0.0f } });
    ASSERT_TRUE(Eventually([this]()
        {
            return Logged(window.Presenter(), "Move done");
        },
        15000));
    frameAndSettle();
    EXPECT_TRUE(FramesAllCorners(*studio, state->FrameBounds().toMap()));

    window.Presenter().SelectJoint(1);
    auto* label = root->findChild<QQuickItem*>("jointLabel");
    auto* ring = root->findChild<QObject*>("selectionRing");
    ASSERT_NE(nullptr, label);
    ASSERT_NE(nullptr, ring);
    EXPECT_TRUE(Eventually([label, ring, state]()
        {
            return label->isVisible() && ring->property("visible").toBool() && state->SelectionText().startsWith("J2 \u00b7 measured");
        }));
    capture("-extended.png");

    auto* focus = MenuAction(window, "Focus Selected Joint");
    ASSERT_NE(nullptr, focus);
    focus->trigger();
    EXPECT_TRUE(Eventually([studio, state]()
        {
            const auto pivot = studio->property("pivot").value<QObject*>()->property("position").value<QVector3D>();
            return (pivot - state->SelectionAnchor()).length() < 0.5f;
        }));

    auto* orthographic = MenuAction(window, "Orthographic");
    ASSERT_NE(nullptr, orthographic);
    orthographic->trigger();
    auto* view = studio->property("view").value<QObject*>();
    EXPECT_EQ(studio->property("orthographicCamera").value<QObject*>(), view->property("camera").value<QObject*>());
    orthographic->trigger();
    EXPECT_EQ(studio->property("camera").value<QObject*>(), view->property("camera").value<QObject*>());

    auto* trailToggle = MenuAction(window, "Tool Trail");
    auto* trail = root->findChild<QObject*>("toolTrail");
    ASSERT_NE(nullptr, trailToggle);
    ASSERT_NE(nullptr, trail);
    ASSERT_TRUE(trail->property("visible").toBool());
    trailToggle->trigger();
    EXPECT_FALSE(trail->property("visible").toBool());
    const QSettings settings{ "embedded-pro", "e-robot-arm Simulator" };
    EXPECT_FALSE(settings.value("overlays/trail", true).toBool());
    trailToggle->trigger();
    EXPECT_TRUE(trail->property("visible").toBool());

    auto* light = MenuAction(window, "Light");
    ASSERT_NE(nullptr, light);
    light->trigger();
    frameAndSettle();
    capture("-extended-light.png");
    MenuAction(window, "Instrument")->trigger();
}

namespace
{
    class SlowAdapterDetector
    {
    public:
        std::function<std::vector<std::string>()> Detector()
        {
            return [this]()
            {
                std::unique_lock lock{ mutex };
                released.wait(lock, [this]()
                    {
                        return open;
                    });
                return std::vector<std::string>{ "can0", "slcan:/dev/ttyACM0" };
            };
        }

        void Release()
        {
            std::scoped_lock lock{ mutex };
            open = true;
            released.notify_all();
        }

        ~SlowAdapterDetector()
        {
            Release();
        }

    private:
        std::mutex mutex;
        std::condition_variable released;
        bool open{ false };
    };

    QLabel& AdapterState(simulator::AdapterDialog& dialog)
    {
        return *dialog.findChild<QLabel*>("adapterSearchState");
    }
}

TEST_F(SimulatorWindowTest, the_adapter_dialog_opens_at_once_and_the_window_stays_live_while_the_search_runs)
{
    SlowAdapterDetector detector;
    window.SetAdapterDetector(detector.Detector());

    QElapsedTimer opening;
    opening.start();
    window.ChooseRealArm();
    EXPECT_LT(opening.elapsed(), 100);

    auto* dialog = window.findChild<simulator::AdapterDialog*>("adapterDialog");
    ASSERT_NE(nullptr, dialog);
    EXPECT_TRUE(dialog->isVisible());
    EXPECT_EQ(simulator::AdapterSearch::searching, dialog->State());
    EXPECT_EQ("Searching for CAN adapters on this computer…", AdapterState(*dialog).text());
    EXPECT_FALSE(dialog->findChild<QPushButton*>("adapterSearchAgain")->isEnabled());

    bool timerFired{ false };
    QTimer::singleShot(50, [&timerFired]()
        {
            timerFired = true;
        });
    EXPECT_TRUE(Eventually([&timerFired]()
        {
            return timerFired;
        },
        1000));

    detector.Release();
    EXPECT_TRUE(Eventually([dialog]()
        {
            return dialog->State() == simulator::AdapterSearch::found;
        }));
    auto* choice = dialog->findChild<QComboBox*>("adapterChoice");
    ASSERT_EQ(2, choice->count());
    EXPECT_EQ("can0", choice->itemText(0));
    dialog->reject();
}

TEST_F(SimulatorWindowTest, an_adapter_search_that_does_not_finish_says_so_and_can_be_searched_again)
{
    SlowAdapterDetector detector;
    window.SetAdapterDetector(detector.Detector());
    window.ChooseRealArm();
    auto* dialog = window.findChild<simulator::AdapterDialog*>("adapterDialog");
    ASSERT_NE(nullptr, dialog);

    EXPECT_TRUE(Eventually([dialog]()
        {
            return dialog->State() == simulator::AdapterSearch::timedOut;
        },
        (simulator::adapterSearchSeconds + 2) * 1000));
    EXPECT_THAT(AdapterState(*dialog).text().toStdString(), testing::HasSubstr("did not finish"));
    auto* searchAgain = dialog->findChild<QPushButton*>("adapterSearchAgain");
    EXPECT_TRUE(searchAgain->isEnabled());

    window.SetAdapterDetector([]()
        {
            return std::vector<std::string>{};
        });
    searchAgain->click();
    detector.Release();
    EXPECT_TRUE(Eventually([dialog]()
        {
            return dialog->State() == simulator::AdapterSearch::noneFound;
        }));
    EXPECT_THAT(AdapterState(*dialog).text().toStdString(), testing::HasSubstr("No CAN adapter detected"));
    dialog->reject();
}

TEST_F(SimulatorWindowTest, cancelling_the_adapter_dialog_drops_a_late_answer_and_switches_nothing)
{
    SlowAdapterDetector detector;
    window.SetAdapterDetector(detector.Detector());
    window.ChooseRealArm();
    QPointer<simulator::AdapterDialog> dialog{ window.findChild<simulator::AdapterDialog*>("adapterDialog") };
    ASSERT_FALSE(dialog.isNull());

    dialog->reject();
    detector.Release();
    QElapsedTimer settle;
    settle.start();
    while (settle.elapsed() < 300)
        QApplication::processEvents(QEventLoop::AllEvents, 10);

    EXPECT_TRUE(dialog.isNull());
    EXPECT_EQ(arm_host::TargetKind::simulated, host.Snapshot().target.kind);
}

int main(int argc, char* argv[])
{
    auto format = QSurfaceFormat::defaultFormat();
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(format);
    QQuickWindow::setGraphicsApi(QSGRendererInterface::OpenGL);

    QApplication application{ argc, argv };
    ui::backend::qt::ApplyTheme(ui::theme::Instrument());
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST_F(SimulatorWindowTest, escape_stops_the_arm_even_while_a_modal_dialog_is_open)
{
    BringUpAndMove(arm::JointPositions{});

    QMessageBox dialog{ QMessageBox::Question, "Re-arm", "Re-arm?", QMessageBox::Cancel, &window };
    dialog.setWindowModality(Qt::ApplicationModal);
    dialog.show();
    ASSERT_TRUE(Eventually([&dialog]()
        {
            return dialog.isVisible() && QApplication::activeModalWidget() == &dialog;
        }));

    QKeyEvent press{ QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier };
    QApplication::sendEvent(&dialog, &press);

    EXPECT_TRUE(Eventually([this]()
        {
            return Logged(window.Presenter(), "mergency stop: accepted");
        }));
}

TEST_F(SimulatorWindowTest, the_single_step_menu_follows_the_rules_and_names_why_an_item_is_disabled)
{
    host.Post(arm_host::BringUp{});
    ASSERT_TRUE(Eventually([this]()
        {
            return window.Presenter().State(simulator::Control::enable).enabled;
        }));

    QMenu* singleStep = nullptr;
    for (auto* menu : window.menuBar()->findChildren<QMenu*>())
        if (menu->title() == "Single &Step")
            singleStep = menu;
    ASSERT_NE(nullptr, singleStep);

    emit singleStep->aboutToShow();
    const auto actions = singleStep->actions();
    ASSERT_EQ(4, actions.size());
    EXPECT_TRUE(actions[0]->isEnabled());
    EXPECT_FALSE(actions[3]->isEnabled());
    EXPECT_EQ("No joint running", actions[3]->toolTip());
}

TEST_F(SimulatorWindowTest, the_tuning_dock_runs_a_step_and_shows_its_metrics_and_plot)
{
    BringUpAndMove(arm::JointPositions{});
    ASSERT_TRUE(Eventually([this]()
        {
            return window.Presenter().State(simulator::Control::move).enabled;
        }));

    auto* dock = window.findChild<QDockWidget*>("tuningDock");
    ASSERT_NE(nullptr, dock);
    dock->raise();
    window.Presenter().SelectJoint(1);
    ASSERT_TRUE(Eventually([dock]()
        {
            return std::ranges::any_of(dock->findChildren<QComboBox*>(), [](const QComboBox* box)
                {
                    return box->currentText() == "J2";
                });
        }));
    auto* run = Button(*dock, "Run");
    auto* exportCsv = Button(*dock, "Export CSV…");
    ASSERT_NE(nullptr, run);
    ASSERT_NE(nullptr, exportCsv);
    ASSERT_TRUE(Eventually([run]()
        {
            return run->isEnabled();
        }))
        << window.Tuning().Hint();
    EXPECT_FALSE(exportCsv->isEnabled());

    run->click();
    ASSERT_TRUE(Eventually([this]()
        {
            return window.Tuning().Status().starts_with("Done");
        },
        10000))
        << window.Tuning().Status() << "\n"
        << LogText(window.Presenter());
    EXPECT_NE(std::string::npos, window.Tuning().Status().find("J2"));
    EXPECT_EQ(ui::theme::StatusLevel::Ok, window.Tuning().StatusLevel());
    EXPECT_TRUE(exportCsv->isEnabled());

    auto* metrics = dock->findChild<QTableWidget*>();
    ASSERT_NE(nullptr, metrics);
    EXPECT_EQ("Rise time", metrics->item(0, 0)->text());
    EXPECT_TRUE(metrics->item(4, 1)->text().endsWith(" rad")) << metrics->item(4, 1)->text().toStdString();
    EXPECT_EQ("—", metrics->item(4, 2)->text());

    if (const auto* path = std::getenv("SIMULATOR_FRAME_CAPTURE"))
    {
        QApplication::processEvents();
        window.grab().save(QString{ path }.replace(".png", "-tuning.png"));
    }
}

TEST_F(SimulatorWindowTest, frame_statistics_report_the_gui_and_render_cost_while_the_arm_moves_and_a_tuning_run_records)
{
    auto onVirtualMotors = false;
#ifdef E_ROBOT_ARM_VIRTUAL_ARM
    if (const auto* engine = std::getenv("E_FOC_VIRTUAL_MOTOR"))
    {
        onVirtualMotors = true;
        window.LoadProfileFrom(VIRTUAL_ARM_PROFILE);
        ASSERT_TRUE(Eventually([this]()
            {
                return Logged(window.Presenter(), "Profile loaded");
            }))
            << LogText(window.Presenter());
        window.Presenter().SwitchTarget({ arm_host::TargetKind::virtualArm, "virtual:arm", engine });
        ASSERT_TRUE(Eventually([this]()
            {
                return host.Snapshot().target.kind == arm_host::TargetKind::virtualArm && host.Snapshot().targetReady;
            },
            30000))
            << LogText(window.Presenter());
    }
#endif

    std::vector<std::string> seconds;
    const auto record = [this, &seconds]()
    {
        if (seconds.empty() || seconds.back() != window.Frames().Text())
            seconds.push_back(window.Frames().Text());
    };
    const auto overrunsBefore = host.Snapshot().overruns;

    const arm::JointPositions target{ foc::Radians{ 0.7f }, foc::Radians{ -0.5f }, foc::Radians{ 0.6f }, foc::Radians{ 0.4f }, foc::Radians{ 0.8f },
        foc::Radians{ 0.0f } };
    host.Post(arm_host::BringUp{});
    if (onVirtualMotors)
    {
        ASSERT_TRUE(Eventually([this, &record]()
            {
                record();
                return Logged(window.Presenter(), "Arm waiting for calibration");
            },
            30000))
            << LogText(window.Presenter());
        host.Post(arm_host::AlignJoints{});
    }
    ASSERT_TRUE(Eventually([this, &record]()
        {
            record();
            return window.Presenter().State(simulator::Control::enable).enabled;
        },
        60000))
        << LogText(window.Presenter());
    host.Post(arm_host::Enable{});
    ASSERT_TRUE(Eventually([this, &record]()
        {
            record();
            return window.Presenter().State(simulator::Control::move).enabled;
        },
        30000))
        << LogText(window.Presenter());
    host.Post(arm_host::MoveTo{ target });
    ASSERT_TRUE(Eventually([this, &record]()
        {
            record();
            return Logged(window.Presenter(), "Move done");
        },
        30000))
        << LogText(window.Presenter());

    auto* dock = window.findChild<QDockWidget*>("tuningDock");
    ASSERT_NE(nullptr, dock);
    dock->raise();
    auto* run = Button(*dock, "Run");
    ASSERT_NE(nullptr, run);
    ASSERT_TRUE(Eventually([run, &record]()
        {
            record();
            return run->isEnabled();
        }));
    run->click();
    ASSERT_TRUE(Eventually([this, &record]()
        {
            record();
            return window.Tuning().Status().starts_with("Done");
        },
        15000))
        << window.Tuning().Status();

    for (const auto& second : seconds)
        std::printf("[METRIC] simulator frame statistics: %s\n", second.c_str());
    std::printf("[METRIC] simulator control-loop overruns during the run: %u\n", host.Snapshot().overruns - overrunsBefore);
    EXPECT_TRUE(std::ranges::any_of(seconds, [](const std::string& second)
        {
            return second.starts_with("3-D ") && second.find("fps") != std::string::npos;
        }));
}

TEST_F(SimulatorWindowTest, the_help_menu_checks_for_updates_and_an_update_waits_while_the_arm_drives_its_joints)
{
    QStringList help;
    for (auto* menu : window.menuBar()->findChildren<QMenu*>())
        if (menu->title() == "&Help")
            for (auto* action : menu->actions())
                help << action->text();
    EXPECT_TRUE(help.contains("Check for &Updates…"));
    EXPECT_TRUE(help.contains("Check for Updates &Automatically"));
    EXPECT_TRUE(help.contains("&Report Issue…"));
    EXPECT_FALSE(window.Updates().State().blocked);

    host.Post(arm_host::BringUp{});
    ASSERT_TRUE(Eventually([this]()
        {
            return window.Presenter().State(simulator::Control::enable).enabled;
        }));
    host.Post(arm_host::Enable{});

    EXPECT_TRUE(Eventually([this]()
        {
            return window.Updates().State().blocked;
        }));
}
