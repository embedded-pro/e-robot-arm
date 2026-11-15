#include "tools/arm_host/ProfileFile.hpp"
#include "tools/virtual_arm/VirtualArm.hpp"
#include <csignal>
#include <ctime>
#include <iostream>
#include <span>
#include <string_view>

namespace
{
    constexpr std::string_view usage{ "usage: e-robot-arm-virtual-arm --profile <file> [--bus <name>] [--engine <e-foc-virtual-motor>] [--work-dir <dir>]" };

    struct Arguments
    {
        std::filesystem::path profile;
        virtual_arm::VirtualArmOptions options;
    };

    std::optional<Arguments> Parse(std::span<char*> words)
    {
        if (words.size() % 2 != 0)
            return std::nullopt;

        Arguments arguments;
        for (std::size_t index = 0; index != words.size(); index += 2)
        {
            const std::string_view name{ words[index] };
            const std::string value{ words[index + 1] };
            if (name == "--profile")
                arguments.profile = value;
            else if (name == "--bus")
                arguments.options.bus = value;
            else if (name == "--engine")
                arguments.options.engine = value;
            else if (name == "--work-dir")
                arguments.options.workDirectory = value;
            else
                return std::nullopt;
        }

        if (arguments.profile.empty())
            return std::nullopt;
        return arguments;
    }

    sigset_t StopSignals()
    {
        sigset_t signals;
        sigemptyset(&signals);
        sigaddset(&signals, SIGINT);
        sigaddset(&signals, SIGTERM);
        return signals;
    }

    bool StopRequested(const sigset_t& signals)
    {
        const timespec wait{ 0, 50'000'000 };
        return sigtimedwait(&signals, nullptr, &wait) > 0;
    }
}

int main(int count, char** words)
{
    const auto arguments = Parse(std::span{ words, static_cast<std::size_t>(count) }.subspan(1));
    if (!arguments.has_value())
    {
        std::cerr << usage << "\n";
        return 2;
    }

    const auto profile = arm_host::ReadProfileFile(arguments->profile);
    if (const auto* problem = std::get_if<std::string>(&profile))
    {
        std::cerr << "profile refused: " << *problem << "\n";
        return 1;
    }

    const auto signals = StopSignals();
    sigprocmask(SIG_BLOCK, &signals, nullptr);

    virtual_arm::VirtualArm arm{ std::get<arm::ArmProfile>(profile).config, arguments->options };
    if (const auto problem = arm.Start())
    {
        std::cerr << "virtual arm not up: " << *problem << "\n";
        return 1;
    }

    std::cout << "virtual arm up on virtual:" << arm.Bus() << "; Ctrl-C stops it" << std::endl;
    bool stopping{ false };
    while (!stopping)
        stopping = StopRequested(signals) || arm.Problem().has_value();

    const auto problem = arm.Problem();
    if (problem.has_value())
        std::cerr << "virtual arm stopped: " << *problem << "\n";
    arm.Stop();
    return problem.has_value() ? 1 : 0;
}
