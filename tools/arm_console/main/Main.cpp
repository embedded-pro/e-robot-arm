#include "can-lite/drivers/hub/VirtualCanHub.hpp"
#include "can-lite/drivers/implementation/PlatformCanAdapter.hpp"
#include "core/arm/Arm.hpp"
#include "e-foc-protocol/client/FocMotorCanClient.hpp"
#include "hal/generic/TimerServiceGeneric.hpp"
#include "infra/event/EventDispatcherThreadAware.hpp"
#include "infra/timer/Timer.hpp"
#include "tools/arm_console/ArmConsole.hpp"
#include "tools/arm_console/ConsoleOptions.hpp"
#include <chrono>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <mutex>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

namespace
{
    using namespace std::chrono_literals;

    arm::ArmConfig ArmConfigFor(const arm::JointNodeIds& nodes)
    {
        arm::ArmConfig config{ nodes, 3, {}, {}, 5ms };
        for (auto& limit : config.positionLimits)
            limit = { foc::Radians{ -std::numbers::pi_v<float> }, foc::Radians{ std::numbers::pi_v<float> } };
        for (auto& limit : config.motionLimits)
            limit = { foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecondSquared{ 2.0f } };
        return config;
    }

    class Session
    {
    public:
        explicit Session(arm_console::ArmConsole& console)
            : console(console)
        {}

        void Push(const std::string& line)
        {
            {
                std::scoped_lock lock{ mutex };
                lines.push_back(line);
            }
            infra::EventDispatcher::Instance().Schedule([this]()
                {
                    ExecutePending();
                });
        }

        bool Ended() const
        {
            return ended;
        }

    private:
        void ExecutePending()
        {
            std::string line;
            {
                std::scoped_lock lock{ mutex };
                if (lines.empty())
                    return;
                line = lines.front();
                lines.pop_front();
            }

            if (!ended && !console.Execute(line))
                ended = true;
        }

        arm_console::ArmConsole& console;
        std::mutex mutex;
        std::deque<std::string> lines;
        bool ended{ false };
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

    static infra::EventDispatcherThreadAware::WithSize<64> eventDispatcher;
    static hal::TimerServiceGeneric timerService;

    services::VirtualCanHub hub;
    if (options->bus.starts_with("virtual:") && !hub.Listen())
        std::cerr << "e-robot-arm-console: joining the virtual CAN hub that is already running\n";

    services::PlatformCanAdapter platformAdapter;
    auto& adapter = platformAdapter.Adapter();
    if (!adapter.Connect(options->bus, options->bitrate))
    {
        std::cerr << "e-robot-arm-console: could not open " << options->bus << "\n";
        return EXIT_FAILURE;
    }

    can::FocMotorCanClient client{ adapter };
    arm::Arm arm{ client.CategoryClient(), ArmConfigFor(options->nodes) };
    arm_console::ArmConsole console{ arm.Safety(), arm.Motion(), std::cout };
    Session session{ console };

    infra::TimerRepeating poll{ 1ms, [&hub, &adapter]()
        {
            hub.Poll();
            adapter.ProcessReadEvent();
        } };
    arm.Loop().Start();

    std::thread reader{ [&session]()
        {
            std::string line;
            while (std::getline(std::cin, line))
                session.Push(line);
            session.Push("quit");
        } };
    reader.detach();

    eventDispatcher.ExecuteUntil([&session]()
        {
            return session.Ended();
        });

    arm.Loop().Stop();
    adapter.Disconnect();
    return EXIT_SUCCESS;
}
