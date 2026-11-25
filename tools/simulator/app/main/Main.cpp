#include "tools/arm_host/AdapterTargetLink.hpp"
#include "tools/arm_host/DefaultArmConfig.hpp"
#ifdef E_ROBOT_ARM_VIRTUAL_ARM
#include "tools/arm_host/VirtualArmTargetLink.hpp"
#endif
#include "tools/simulator/app/Diagnostics.hpp"
#include "tools/simulator/app/SimulatorWindow.hpp"
#include "ui/backend/qt/QtTheme.hpp"
#include "ui/backend/qt/update/CrashReporter.hpp"
#include "ui/backend/qt/update/IssueLog.hpp"
#include <QApplication>
#include <QQuickWindow>
#include <QSurfaceFormat>
#include <QtQml/QQmlExtensionPlugin>
#include <chrono>
#include <span>

Q_IMPORT_QML_PLUGIN(SimulatorPlugin)
Q_IMPORT_QML_PLUGIN(UiQuick3DPlugin)

int main(int argc, char* argv[])
{
    // Before anything else: the Windows build has no console, so the log is the only trace of a failed start.
    QCoreApplication::setOrganizationName("embedded-pro");
    QCoreApplication::setApplicationName("e-robot-arm Simulator");
    const std::span<char* const> arguments{ argv, static_cast<std::size_t>(argc) };
    ui::backend::qt::update::IssueLog issueLog{ simulator::IssueLogConfig() };
    issueLog.CaptureQtMessages();
    const ui::backend::qt::update::CrashReporter crashReporter{ issueLog };
    simulator::LogStart(issueLog, arguments);
    const auto options = simulator::ParseStartOptions(arguments);

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
    // The Windows 11 style draws spin box and combo box arrows invisible under a custom palette.
    QApplication::setStyle("Fusion");
    ui::backend::qt::ApplyTheme(ui::theme::Instrument());

    const auto profile = arm_host::DefaultArmProfile(arm_host::defaultNodes);
    const auto start = std::chrono::system_clock::now();
    constexpr uint32_t bitrate{ 1000000 };
    auto opener = arm_host::HostTargetOpener(arm_host::simulatedTimeConstant, bitrate);
#ifdef E_ROBOT_ARM_VIRTUAL_ARM
    opener = arm_host::VirtualArmTargetOpener(std::move(opener), bitrate);
#endif
    issueLog.Write(ui::backend::qt::update::IssueSeverity::info, "Starting the arm host");
    arm_host::ArmHost host{ profile, opener, {} };
    simulator::SimulatorWindow window{ host, profile.config.positionLimits, start, issueLog, options };
    window.show();
    window.StartUpdateChecks();

    const auto result = QApplication::exec();
    issueLog.Write(ui::backend::qt::update::IssueSeverity::info, QString{ "Closed with exit code %1" }.arg(result));
    return result;
}
