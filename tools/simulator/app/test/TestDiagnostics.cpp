#include "tools/simulator/app/Diagnostics.hpp"
#include "gmock/gmock.h"
#include <QDir>
#include <QTemporaryDir>
#include <array>

namespace
{
    class DiagnosticsTest
        : public testing::Test
    {
    public:
        ui::backend::qt::update::IssueLog::Config Config() const
        {
            auto config = simulator::IssueLogConfig();
            config.directory = temporary.path();
            return config;
        }

        QTemporaryDir temporary;
    };
}

TEST(StartOptionsTest, safe_start_is_off_without_the_flag)
{
    std::array<char*, 2> arguments{ const_cast<char*>("e-robot-arm-simulator"), const_cast<char*>("--other") };

    EXPECT_FALSE(simulator::ParseStartOptions(arguments).safeStart);
}

TEST(StartOptionsTest, safe_start_is_on_with_the_flag)
{
    std::array<char*, 2> arguments{ const_cast<char*>("e-robot-arm-simulator"), const_cast<char*>("--safe-start") };

    EXPECT_TRUE(simulator::ParseStartOptions(arguments).safeStart);
}

TEST(StartOptionsTest, the_log_captures_every_qt_message_into_the_logs_folder)
{
    const auto config = simulator::IssueLogConfig();

    EXPECT_EQ("simulator", config.baseName);
    EXPECT_EQ(QtDebugMsg, config.minimumQtMessage);
    EXPECT_EQ("logs", QDir{ config.directory }.dirName());
}

TEST_F(DiagnosticsTest, the_start_line_names_the_version_platform_qt_and_arguments)
{
    ui::backend::qt::update::IssueLog log{ Config() };
    std::array<char*, 2> arguments{ const_cast<char*>("e-robot-arm-simulator"), const_cast<char*>("--safe-start") };

    simulator::LogStart(log, arguments);

    const auto lines = log.RecentLines(10);
    ASSERT_EQ(1, lines.size());
    EXPECT_THAT(lines.front().toStdString(), testing::HasSubstr("[INFO ] Starting e-robot-arm Simulator " E_ROBOT_ARM_VERSION));
    EXPECT_THAT(lines.front().toStdString(), testing::HasSubstr(", Qt " QT_VERSION_STR));
    EXPECT_THAT(lines.front().toStdString(), testing::HasSubstr("arguments: --safe-start"));
}
