#include "core/blinky_cli/BlinkyCli.hpp"
#include <array>

namespace application
{
    BlinkyCli::BlinkyCli(platform::Platform& platform)
        : debugLed{ platform.StatusLed() }
        , terminal{ platform.Communication(), platform.Tracer() }
        , commands{ terminal, platform.Tracer() }
    {
        platform.Tracer().Trace() << "embedded-scaffold blinky-cli ready - try 'ping' or 'id'";
    }

    BlinkyCli::CliCommands::CliCommands(services::TerminalWithCommands& terminal, services::Tracer& tracer)
        : services::TerminalCommands(terminal)
        , tracer(tracer)
    {}

    infra::MemoryRange<const services::TerminalCommands::Command> BlinkyCli::CliCommands::Commands()
    {
        static const std::array<Command, 2> commands = { {
            { { "ping", "p", "reply with pong" },
                [this](const infra::BoundedConstString& params)
                {
                    Ping(params);
                } },
            { { "id", "i", "print the board identifier" },
                [this](const infra::BoundedConstString& params)
                {
                    Identify(params);
                } },
        } };

        return infra::MakeRange(commands);
    }

    void BlinkyCli::CliCommands::Ping(const infra::BoundedConstString&)
    {
        tracer.Trace() << "pong";
    }

    void BlinkyCli::CliCommands::Identify(const infra::BoundedConstString&)
    {
        tracer.Trace() << "embedded-scaffold blinky-cli";
    }
}
