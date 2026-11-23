#include "tools/arm_console/ConsoleSession.hpp"
#include "tools/arm_host/ArmCommand.hpp"

namespace arm_console
{
    ConsoleSession::ConsoleSession(ArmConsole& console, CommandTracker& tracker, std::ostream& output, bool batch)
        : console(console)
        , tracker(tracker)
        , output(output)
        , batch(batch)
    {}

    bool ConsoleSession::Handle(const std::string& line)
    {
        ++lineNumber;
        const auto result = console.Execute(line);
        if (result.outcome == Outcome::quit)
            return false;
        if (Succeeded(result, line))
            return true;
        return Stop(line);
    }

    int ConsoleSession::ExitStatus() const
    {
        return batch && failed ? 1 : 0;
    }

    bool ConsoleSession::Succeeded(const ExecuteResult& result, const std::string& line)
    {
        switch (result.outcome)
        {
            case Outcome::failed:
                return false;
            case Outcome::posted:
            {
                const auto answer = tracker.Await(result.command, answerTimeout);
                if (answer == Answer::noAnswer)
                    output << "no answer in 2 s: " << line << "\n";
                return answer == Answer::accepted;
            }
            case Outcome::wait:
                return result.waitState.has_value() ? WaitForState(*result.waitState, result.waitFor) : WaitForMove(result.waitFor);
            default:
                return true;
        }
    }

    bool ConsoleSession::WaitForMove(std::chrono::milliseconds timeout)
    {
        const auto end = tracker.AwaitMove(timeout);
        if (!end.has_value())
            output << "wait: move still running after " << std::chrono::duration<double>{ timeout }.count() << " s\n";
        return end == MoveEnd::done;
    }

    bool ConsoleSession::WaitForState(arm::ArmState wanted, std::chrono::milliseconds timeout)
    {
        const auto waited = tracker.AwaitState(wanted, timeout);
        if (!waited.reached)
            output << "wait: arm " << arm_host::StateName(waited.state) << ", not " << arm_host::StateName(wanted) << "\n";
        return waited.reached;
    }

    bool ConsoleSession::Stop(const std::string& line)
    {
        failed = true;
        if (!batch)
            return true;
        output << "batch stopped at line " << lineNumber << ": " << line << "\n";
        return false;
    }
}
