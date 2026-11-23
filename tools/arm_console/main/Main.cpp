#include "tools/arm_console/ArmConsole.hpp"
#include "tools/arm_console/CommandTracker.hpp"
#include "tools/arm_console/ConsoleOptions.hpp"
#include "tools/arm_console/ConsoleSession.hpp"
#include "tools/arm_host/AdapterTargetLink.hpp"
#include "tools/arm_host/ArmHost.hpp"
#include "tools/arm_host/DefaultArmConfig.hpp"
#include "tools/arm_host/ProfileFile.hpp"
#ifdef E_ROBOT_ARM_VIRTUAL_ARM
#include "tools/virtual_arm/VirtualArm.hpp"
#endif
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace
{
    using namespace std::chrono_literals;

    arm_host::ArmTarget TargetFor(const std::string& bus)
    {
        if (bus == "simulated")
            return { arm_host::TargetKind::simulated, bus };
        if (bus.starts_with("virtual:"))
            return { arm_host::TargetKind::virtualBus, bus };
        return { arm_host::TargetKind::real, bus };
    }

    class EventPrinter
    {
    public:
        EventPrinter(arm_host::ArmHost& host, arm_console::ArmConsole& console, arm_console::CommandTracker& tracker)
            : host(host)
            , console(console)
            , tracker(tracker)
        {}

        void Run(const std::stop_token& stop)
        {
            while (!stop.stop_requested())
            {
                Print();
                std::this_thread::sleep_for(20ms);
            }
            Print();
        }

    private:
        void Print()
        {
            for (const auto& sample : host.TakeTrace())
                console.Trace(sample);
            for (const auto& event : host.TakeEvents())
            {
                console.Print(event);
                std::cout.flush();
                tracker.Note(event);
            }
            std::cout.flush();
        }

        arm_host::ArmHost& host;
        arm_console::ArmConsole& console;
        arm_console::CommandTracker& tracker;
    };
}

int main(int argc, const char* const argv[])
{
    const auto options = arm_console::ParseOptions(std::vector<std::string>(argv, argv + argc));
    if (!options)
    {
        std::cerr << arm_console::OptionsUsage() << "\n";
        return EXIT_FAILURE;
    }

    auto profile = arm_host::DefaultArmProfile(options->nodes);
    const auto& profilePath = options->virtualArm.empty() ? options->profile : options->virtualArm;
    if (!profilePath.empty())
    {
        auto read = arm_host::ReadProfileFile(profilePath);
        if (const auto* problem = std::get_if<std::string>(&read))
        {
            std::cerr << "profile refused: " << *problem << "\n";
            return EXIT_FAILURE;
        }
        profile = std::get<arm::ArmProfile>(read);
    }

#ifdef E_ROBOT_ARM_VIRTUAL_ARM
    std::optional<virtual_arm::VirtualArm> virtualArm;
    if (!options->virtualArm.empty())
    {
        virtual_arm::VirtualArmOptions virtualArmOptions;
        virtualArmOptions.bus = options->bus.substr(std::string_view{ "virtual:" }.size());
        if (!options->engine.empty())
            virtualArmOptions.engine = options->engine;
        virtualArm.emplace(profile.config, virtualArmOptions);
        if (const auto problem = virtualArm->Start())
        {
            std::cerr << "virtual arm not up: " << *problem << "\n";
            return EXIT_FAILURE;
        }
        std::cout << "virtual arm up on " << options->bus << "\n";
    }
#else
    if (!options->virtualArm.empty())
    {
        std::cerr << "--virtual-arm needs a Linux host\n";
        return EXIT_FAILURE;
    }
#endif

    arm_host::ArmHost host{ profile, arm_host::HostTargetOpener(arm_host::simulatedTimeConstant, options->bitrate), TargetFor(options->bus) };
    arm_console::ArmConsole console{ host, std::cout };
    arm_console::CommandTracker tracker;
    EventPrinter printer{ host, console, tracker };
    arm_console::ConsoleSession session{ console, tracker, std::cout, options->batch };

    std::jthread printing{ [&printer](const std::stop_token& stop)
        {
            printer.Run(stop);
        } };

    std::string line;
    while (std::getline(std::cin, line) && session.Handle(line))
        std::cout.flush();

    printing.request_stop();
    printing.join();
    return session.ExitStatus();
}
