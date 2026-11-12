#include "tools/arm_host/AdapterTargetLink.hpp"
#include "tools/arm_host/DefaultArmConfig.hpp"
#include "tools/simulator/app/SimulatorWindow.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include "gmock/gmock.h"
#include <QApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QKeyEvent>
#include <QMessageBox>
#include <QQuickItem>
#include <QQuickWidget>
#include <QQuickWindow>
#include <QSurfaceFormat>
#include <QtQml/QQmlExtensionPlugin>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <functional>

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

    bool HasDetail(const QImage& image)
    {
        const auto reference = image.pixel(0, 0);
        int differing = 0;
        for (int y = 0; y < image.height(); y += 8)
            for (int x = 0; x < image.width(); x += 8)
                differing += image.pixel(x, y) != reference ? 1 : 0;
        return differing > 100;
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
            host.Post(arm_host::CheckContracts{});
            ASSERT_TRUE(Eventually([this]()
                {
                    return window.Presenter().State(simulator::Control::positionMode).enabled;
                }));
            host.Post(arm_host::SelectMode{ can::FocMotorMode::position });
            host.Post(arm_host::StartJoints{});
            ASSERT_TRUE(Eventually([this]()
                {
                    return window.Presenter().State(simulator::Control::move).enabled;
                }));
            host.Post(arm_host::MoveTo{ target });
        }

        arm::ArmConfig config{ arm_host::DefaultArmConfig(arm_host::defaultNodes) };
        infra::TimePoint start{ std::chrono::system_clock::now() };
        arm_host::ArmHost host{ config, arm_host::HostTargetOpener(arm_host::defaultNodes, arm_host::simulatedTimeConstant, 1000000), {} };
        simulator::SimulatorWindow window{ host, config.positionLimits, start };
    };
}

TEST_F(SimulatorWindowTest, loads_the_lit_scene_and_shows_a_move_with_its_ghost)
{
    ASSERT_EQ(QQuickWidget::Ready, window.ArmView().status());
    ASSERT_NE(nullptr, window.ArmView().rootObject());

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
