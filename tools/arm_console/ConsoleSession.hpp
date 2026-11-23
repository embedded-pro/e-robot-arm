#pragma once

#include "tools/arm_console/ArmConsole.hpp"
#include "tools/arm_console/CommandTracker.hpp"
#include <chrono>
#include <cstdint>
#include <ostream>
#include <string>

namespace arm_console
{
    class ConsoleSession
    {
    public:
        static constexpr std::chrono::milliseconds answerTimeout{ 2000 };

        ConsoleSession(ArmConsole& console, CommandTracker& tracker, std::ostream& output, bool batch);

        bool Handle(const std::string& line);
        int ExitStatus() const;

    private:
        bool Succeeded(const ExecuteResult& result, const std::string& line);
        bool WaitForMove(std::chrono::milliseconds timeout);
        bool WaitForState(arm::ArmState wanted, std::chrono::milliseconds timeout);
        bool Stop(const std::string& line);

        ArmConsole& console;
        CommandTracker& tracker;
        std::ostream& output;
        bool batch;
        uint32_t lineNumber{ 0 };
        bool failed{ false };
    };
}
