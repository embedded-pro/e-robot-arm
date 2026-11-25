#include "tools/simulator/app/Diagnostics.hpp"
#include <QDir>
#include <QStandardPaths>
#include <QSysInfo>
#include <cstring>

namespace simulator
{
    StartOptions ParseStartOptions(std::span<char* const> arguments)
    {
        StartOptions options;
        for (const auto* argument : arguments.subspan(arguments.empty() ? 0 : 1))
            if (std::strcmp(argument, "--safe-start") == 0)
                options.safeStart = true;
        return options;
    }

    ui::backend::qt::update::IssueLog::Config IssueLogConfig()
    {
        ui::backend::qt::update::IssueLog::Config config;
        config.directory = QDir{ QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation) }.filePath("logs");
        config.baseName = "simulator";
        config.minimumQtMessage = QtDebugMsg;
        return config;
    }

    void LogStart(ui::backend::qt::update::IssueLog& log, std::span<char* const> arguments)
    {
        QStringList passed;
        for (const auto* argument : arguments.subspan(arguments.empty() ? 0 : 1))
            passed.append(QString::fromLocal8Bit(argument));
        log.Write(ui::backend::qt::update::IssueSeverity::info,
            QString{ "Starting e-robot-arm Simulator %1 on %2 (%3), Qt %4; arguments: %5" }
                .arg(E_ROBOT_ARM_VERSION, QSysInfo::prettyProductName(), QSysInfo::currentCpuArchitecture(), qVersion(), passed.isEmpty() ? "none" : passed.join(' ')));
    }
}
