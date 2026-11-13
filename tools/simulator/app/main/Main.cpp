#include "tools/arm_host/AdapterTargetLink.hpp"
#include "tools/arm_host/DefaultArmConfig.hpp"
#include "tools/simulator/app/SimulatorWindow.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include <QApplication>
#include <QQuickWindow>
#include <QSurfaceFormat>
#include <QtQml/QQmlExtensionPlugin>
#include <chrono>

Q_IMPORT_QML_PLUGIN(SimulatorPlugin)

int main(int argc, char* argv[])
{
    auto format = QSurfaceFormat::defaultFormat();
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(format);
#if defined(Q_OS_WIN)
    QQuickWindow::setGraphicsApi(QSGRendererInterface::Direct3D11);
#else
    QQuickWindow::setGraphicsApi(QSGRendererInterface::OpenGL);
#endif

    QApplication application{ argc, argv };
    QApplication::setApplicationName("e-robot-arm Simulator");
    QApplication::setOrganizationName("embedded-pro");
    ui::backend::qt::ApplyTheme(ui::theme::Instrument());

    const auto profile = arm_host::DefaultArmProfile(arm_host::defaultNodes);
    const auto start = std::chrono::system_clock::now();
    arm_host::ArmHost host{ profile, arm_host::HostTargetOpener(arm_host::simulatedTimeConstant, 1000000), {} };
    simulator::SimulatorWindow window{ host, profile.config.positionLimits, start };
    window.show();

    return QApplication::exec();
}
