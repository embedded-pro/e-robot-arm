#include "tools/simulator/view_models/EventLog.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include "tools/simulator/view_models/JointRows.hpp"
#include <array>
#include <chrono>
#include <format>

namespace simulator
{
    namespace
    {
        std::string Joint(std::size_t joint)
        {
            return std::format("Joint {}", joint + 1);
        }

        std::string Text(std::string_view text)
        {
            return std::string{ text };
        }

        std::string SentenceCase(std::string_view text)
        {
            std::string sentence{ text };
            if (!sentence.empty() && sentence.front() >= 'a' && sentence.front() <= 'z')
                sentence.front() = static_cast<char>(sentence.front() - 'a' + 'A');
            return sentence;
        }

        std::pair<LogLevel, std::string> Message(const arm_host::ArmEvent& event)
        {
            using enum arm_host::ArmEventKind;
            switch (event.kind)
            {
                case accepted:
                    return { LogLevel::info, std::format("{}: accepted", SentenceCase(event.commandName)) };
                case refused:
                    return { LogLevel::warning, std::format("{} refused: {}", SentenceCase(event.commandName), event.reason) };
                case contractsResolved:
                    return event.contracts == can::ContractCompatibility::compatible ? std::pair{ LogLevel::info, Text("Contracts compatible") }
                                                                                     : std::pair{ LogLevel::fault, Text("Contracts incompatible") };
                case jointFault:
                    return { LogLevel::fault, Joint(event.joint) + " fault: " + Text(FaultName(event.fault)) };
                case jointSilent:
                    return { LogLevel::fault, Joint(event.joint) + " silent" };
                case tripped:
                    return { LogLevel::fault, "Tripped by " + Joint(event.joint) };
                case moveDone:
                    return { LogLevel::info, "Move done" };
                case moveAborted:
                    return { LogLevel::warning, "Move aborted" };
                case targetReady:
                    return { LogLevel::info, "Target ready" };
                case targetFailed:
                    return { LogLevel::fault, "Target switch failed: " + Text(event.reason) };
                case stateChanged:
                    if (event.joint != arm_host::noJoint)
                        return { LogLevel::fault, "Arm " + Text(arm_host::StateName(event.state)) + ": " + Joint(event.joint) + " " + Text(event.reason) };
                    return { LogLevel::info, "Arm " + Text(arm_host::StateName(event.state)) };
                case controllerApplied:
                    return { LogLevel::info, Joint(event.joint) + " controller applied" };
                case controllerRead:
                    return { LogLevel::info, Joint(event.joint) + " controller read" };
                case controllerFailed:
                    return { LogLevel::fault, Joint(event.joint) + " controller failed: " + Text(event.reason) };
                case profileLoaded:
                    return { LogLevel::info, "Profile loaded: the arm was rebuilt" };
            }
            return { LogLevel::info, "Unknown event" };
        }
    }

    LogLine Describe(const arm_host::ArmEvent& event, infra::TimePoint start)
    {
        auto [level, message] = Message(event);
        const auto seconds = std::chrono::duration<float>(event.time - start).count();
        return { seconds, level, event.joint, std::move(message) };
    }

    EventLog::EventLog(infra::TimePoint start, std::size_t capacity)
        : start(start)
        , capacity(capacity)
    {}

    void EventLog::Add(const arm_host::ArmEvent& event)
    {
        Add(Describe(event, start));
    }

    void EventLog::Add(LogLine line)
    {
        if (lines.size() == capacity)
            lines.pop_front();
        lines.push_back(std::move(line));
    }

    const std::deque<LogLine>& EventLog::Lines() const
    {
        return lines;
    }

    bool EventLog::Passes(const LogLine& line, const LogFilter& filter) const
    {
        if (line.level < filter.minimum)
            return false;
        return !filter.joint.has_value() || line.joint == *filter.joint;
    }

    std::string EventLog::Format(const LogLine& line) const
    {
        static constexpr std::array<const char*, 3> levels{ "Info", "Warning", "Fault" };
        const auto joint = line.joint == arm_host::noJoint ? std::string{ "arm" } : std::format("J{}", line.joint + 1);
        return std::format("{:10.3f}  {}  {}  {}", line.seconds, levels[static_cast<std::size_t>(line.level)], joint, line.message);
    }

    std::string EventLog::Text(const LogFilter& filter) const
    {
        std::string text;
        for (const auto& line : lines)
            if (Passes(line, filter))
                text += Format(line) + "\n";
        return text;
    }
}
