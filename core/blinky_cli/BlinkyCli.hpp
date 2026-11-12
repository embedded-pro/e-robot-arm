#ifndef CORE_BLINKY_CLI_BLINKY_CLI_HPP
#define CORE_BLINKY_CLI_BLINKY_CLI_HPP

#include "core/platform_abstraction/Platform.hpp"
#include "services/peripheral/DebugLed.hpp"
#include "services/util/Terminal.hpp"

namespace application
{
    // Portable application logic: blinks the status LED and serves a small
    // command-line interface over the platform's serial channel.
    //
    // It depends ONLY on platform::Platform, so it is compiled once and runs on
    // every target (TI, ST) and on the host, where it is unit-tested against a
    // PlatformMock. The board-specific wiring lives in
    // targets/platform_implementations/<platform>.
    class BlinkyCli
    {
    public:
        explicit BlinkyCli(platform::Platform& platform);

    private:
        class CliCommands final
            : public services::TerminalCommands
        {
        public:
            CliCommands(services::TerminalWithCommands& terminal, services::Tracer& tracer);

            infra::MemoryRange<const Command> Commands() override;

        private:
            void Ping(const infra::BoundedConstString& params);
            void Identify(const infra::BoundedConstString& params);

            services::Tracer& tracer;
        };

        services::DebugLed debugLed;
        services::TerminalWithCommandsImpl::WithMaxQueueAndMaxHistory<> terminal;
        CliCommands commands;
    };
}

#endif
