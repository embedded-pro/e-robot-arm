#include "tools/arm_host/AdapterTargetLink.hpp"
#include "tools/arm_host/DefaultArmConfig.hpp"
#include "tools/arm_host/ProfileFile.hpp"
#include "tools/simulator/app/SimulatorWindow.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include "gmock/gmock.h"
#include <QApplication>
#include <QComboBox>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QQuickItem>
#include <QQuickWidget>
#include <QQuickWindow>
#include <QSettings>
#include <QSurfaceFormat>
#include <QTableWidget>
#include <QtQml/QQmlExtensionPlugin>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <string>

Q_IMPORT_QML_PLUGIN(SimulatorPlugin)

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
                }));
            host.Post(arm_host::MoveTo{ target });
            ASSERT_TRUE(Eventually([this]()
                {
                    return Logged(window.Presenter(), "Move: accepted");
                }))
                << LogText(window.Presenter());
        }

        arm::ArmProfile profile{ arm_host::DefaultArmProfile(arm_host::defaultNodes) };
        infra::TimePoint start{ std::chrono::system_clock::now() };
        arm_host::ArmHost host{ profile, arm_host::HostTargetOpener(arm_host::simulatedTimeConstant, 1000000), {} };
        simulator::SimulatorWindow window{ host, profile.config.positionLimits, start };
    };
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

    int frames = 0;
    QElapsedTimer rendering;
    qint64 renderNanoseconds = 0;
    auto* quickWindow = window.ArmView().quickWindow();
    const auto before = QObject::connect(quickWindow, &QQuickWindow::beforeRendering, [&rendering]()
        {
            rendering.start();
        });
    const auto after = QObject::connect(quickWindow, &QQuickWindow::afterRendering, [&frames, &renderNanoseconds, &rendering]()
        {
            ++frames;
            renderNanoseconds += rendering.nsecsElapsed();
        });
    QElapsedTimer measuring;
    measuring.start();
    EXPECT_TRUE(Eventually([this]()
        {
            return window.Presenter().Scene().GhostVisible() && window.Presenter().Scene().MeasuredDegrees()[0] > 20.0f;
        }));
    const auto elapsed = measuring.elapsed();
    QObject::disconnect(before);
    QObject::disconnect(after);

    const auto frame = window.ArmView().grabFramebuffer();
    ASSERT_FALSE(frame.isNull());
    EXPECT_TRUE(HasDetail(frame));
    EXPECT_GT(frames, 0);
    std::printf("[METRIC] simulator frames per second while moving: %.1f\n", frames * 1000.0 / static_cast<double>(std::max<qint64>(elapsed, 1)));
    std::printf("[METRIC] simulator mean scene-graph render call per frame: %.2f ms\n", frames == 0 ? 0.0 : static_cast<double>(renderNanoseconds) / frames / 1.0e6);

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
}

TEST_F(SimulatorWindowTest, an_injected_fault_trips_the_arm_and_names_its_source)
{
    BringUpAndMove(arm::JointPositions{});
    window.Presenter().InjectFault(2);

    EXPECT_TRUE(Eventually([this]()
        {
            return window.Presenter().Status().trip.find("Joint 3") != std::string::npos;
        }));
    EXPECT_EQ(ui::theme::StatusLevel::Fault, window.Presenter().Status().tripLevel);
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
