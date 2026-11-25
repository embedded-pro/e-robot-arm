#pragma once

#include "ui/backend/qt/update/IssueLog.hpp"
#include <span>

namespace simulator
{
    struct StartOptions
    {
        bool safeStart{ false };
    };

    StartOptions ParseStartOptions(std::span<char* const> arguments);

    // Needs the organisation and application names set; QApplication need not exist yet.
    ui::backend::qt::update::IssueLog::Config IssueLogConfig();

    void LogStart(ui::backend::qt::update::IssueLog& log, std::span<char* const> arguments);
}
