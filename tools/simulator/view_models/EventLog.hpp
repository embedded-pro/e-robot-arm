#ifndef TOOLS_SIMULATOR_VIEW_MODELS_EVENT_LOG_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_EVENT_LOG_HPP

#include "tools/arm_host/ArmEvent.hpp"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace simulator
{
    enum class LogLevel : uint8_t
    {
        info,
        warning,
        fault
    };

    struct LogLine
    {
        float seconds;
        LogLevel level;
        std::size_t joint;
        std::string message;
    };

    struct LogFilter
    {
        LogLevel minimum{ LogLevel::info };
        std::optional<std::size_t> joint{};
    };

    LogLine Describe(const arm_host::ArmEvent& event, infra::TimePoint start);

    class EventLog
    {
    public:
        static constexpr std::size_t defaultCapacity{ 10000 };

        explicit EventLog(infra::TimePoint start, std::size_t capacity = defaultCapacity);

        void Add(const arm_host::ArmEvent& event);
        void Add(LogLine line);
        const std::deque<LogLine>& Lines() const;
        bool Passes(const LogLine& line, const LogFilter& filter) const;
        std::string Format(const LogLine& line) const;
        std::string Text(const LogFilter& filter) const;

    private:
        infra::TimePoint start;
        std::size_t capacity;
        std::deque<LogLine> lines;
    };
}

#endif
