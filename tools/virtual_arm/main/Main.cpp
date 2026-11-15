#include "tools/arm_host/ProfileFile.hpp"
#include "tools/virtual_arm/VirtualArm.hpp"
#include <atomic>
#include <csignal>
#include <iostream>
#include <string_view>
#include <thread>

namespace
{
    std::atomic<bool> interrupted{ false };

    void Interrupt(int)
    {
        interrupted = true;
    }

    constexpr std::string_view usage{ "usage: e-robot-arm-virtual-arm --profile <file> [--bus <name>] [--engine <e-foc-virtual-motor>] [--work-dir <dir>]" };
}

int main(int count, char** arguments)
{
    std::filesystem::path profilePath;
    virtual_arm::VirtualArmOptions options;
    for (int index = 1; index < count; ++index)
    {
        const std::string_view name{ arguments[index] };
        if (index + 1 == count)
        {
            std::cerr << usage << "\n";
            return 2;
        }

        const std::string value{ arguments[++index] };
        if (name == "--profile")
            profilePath = value;
        else if (name == "--bus")
            options.bus = value;
        else if (name == "--engine")
            options.engine = value;
        else if (name == "--work-dir")
            options.workDirectory = value;
        else
        {
            std::cerr << usage << "\n";
            return 2;
        }
    }

    if (profilePath.empty())
    {
        std::cerr << usage << "\n";
        return 2;
    }

    const auto profile = arm_host::ReadProfileFile(profilePath);
    if (const auto* problem = std::get_if<std::string>(&profile))
    {
        std::cerr << "profile refused: " << *problem << "\n";
        return 1;
    }

    std::signal(SIGINT, Interrupt);
    std::signal(SIGTERM, Interrupt);
    virtual_arm::VirtualArm arm{ std::get<arm::ArmProfile>(profile).config, options };
    if (const auto problem = arm.Start())
    {
        std::cerr << "virtual arm not up: " << *problem << "\n";
        return 1;
    }

    std::cout << "virtual arm up on virtual:" << arm.Bus() << "; Ctrl-C stops it" << std::endl;
    while (!interrupted && !arm.Problem().has_value())
        std::this_thread::sleep_for(std::chrono::milliseconds{ 50 });

    if (const auto problem = arm.Problem())
        std::cerr << "virtual arm stopped: " << *problem << "\n";
    arm.Stop();
    return arm.Problem().has_value() ? 1 : 0;
}
