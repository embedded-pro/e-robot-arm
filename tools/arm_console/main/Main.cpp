#include "tools/arm_console/ArmConsole.hpp"
#include "tools/arm_console/ConsoleOptions.hpp"
#include "tools/arm_host/AdapterTargetLink.hpp"
#include "tools/arm_host/ArmHost.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <numbers>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace
{
    using namespace std::chrono_literals;

    arm::ArmConfig ArmConfigFor(const arm::JointNodeIds& nodes)
    {
        arm::ArmConfig config{ nodes, 3, {}, {}, 10ms };
        for (auto& limit : config.positionLimits)
            limit = { foc::Radians{ -std::numbers::pi_v<float> }, foc::Radians{ std::numbers::pi_v<float> } };
        for (auto& limit : config.motionLimits)
            limit = { foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecondSquared{ 2.0f } };
        return config;
    }

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
        EventPrinter(arm_host::ArmHost& host, arm_console::ArmConsole& console)
            : host(host)
            , console(console)
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

        void WaitForAnswer(uint32_t command)
        {
            std::unique_lock lock{ mutex };
            answeredCondition.wait_for(lock, 2s, [this, command]()
                {
                    return answered.contains(command);
                });
        }

    private:
        void Print()
        {
            for (const auto& event : host.TakeEvents())
            {
                std::scoped_lock lock{ mutex };
                console.Print(event);
                if (event.kind == arm_host::ArmEventKind::accepted || event.kind == arm_host::ArmEventKind::refused)
                    answered.insert(event.command);
                answeredCondition.notify_all();
            }
            std::cout.flush();
        }

        arm_host::ArmHost& host;
        arm_console::ArmConsole& console;
        std::mutex mutex;
        std::condition_variable answeredCondition;
        std::set<uint32_t> answered;
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

    arm_host::ArmHost host{ ArmConfigFor(options->nodes), arm_host::HostTargetOpener(options->nodes, 50ms, options->bitrate), TargetFor(options->bus) };
    arm_console::ArmConsole console{ host, std::cout };
    EventPrinter printer{ host, console };

    std::jthread printing{ [&printer](const std::stop_token& stop)
        {
            printer.Run(stop);
        } };

    std::string line;
    while (std::getline(std::cin, line))
    {
        const auto result = console.Execute(line);
        if (result.outcome == arm_console::Outcome::quit)
            break;
        if (result.outcome == arm_console::Outcome::posted)
            printer.WaitForAnswer(result.command);
    }

    return EXIT_SUCCESS;
}
