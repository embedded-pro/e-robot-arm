#include "can-lite/drivers/hub/VirtualCanHub.hpp"
#include "can-lite/drivers/implementation/PlatformCanAdapter.hpp"
#include "core/arm/Arm.hpp"
#include "e-foc-protocol/client/FocMotorCanClient.hpp"
#include "hal/generic/TimerServiceGeneric.hpp"
#include "infra/event/EventDispatcherThreadAware.hpp"
#include "infra/timer/Timer.hpp"
#include "tools/arm_console/ArmConsole.hpp"
#include "tools/arm_console/ConsoleOptions.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <numbers>
#include <string>
#include <string_view>
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
        Session(arm_console::ArmConsole& console, const infra::Function<void()>& onEnded)
            : console(console)
            , onEnded(onEnded)
        {}

        void Execute(std::string_view line)
        {
            std::unique_lock lock{ mutex };
            pending = std::string{ line };
            done = false;
            infra::EventDispatcher::Instance().Schedule([this]()
                {
                    ExecutePending();
                });
            executed.wait(lock, [this]()
                {
                    return done;
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
                line = pending;
            }

            if (!console.Execute(line))
            {
                onEnded();
                ended = true;
            }

            {
                std::scoped_lock lock{ mutex };
                done = true;
            }
            executed.notify_one();
        }

        arm_console::ArmConsole& console;
        infra::Function<void()> onEnded;
        std::mutex mutex;
        std::condition_variable executed;
        std::string pending;
        bool done{ false };
        std::atomic<bool> ended{ false };
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

    infra::TimerRepeating poll{ 1ms, [&hub, &adapter]()
        {
            hub.Poll();
            adapter.ProcessReadEvent();
        } };
    arm.Loop().Start();

    Session session{ console, [&poll, &arm]()
        {
            arm.Loop().Stop();
            poll.Cancel();
        } };

    {
        std::jthread eventLoop{ [&session]()
            {
                eventDispatcher.ExecuteUntil([&session]()
                    {
                        return session.Ended();
                    });
            } };

        std::string line;
        while (!session.Ended() && std::getline(std::cin, line))
            session.Execute(line);

        if (!session.Ended())
            session.Execute("quit");
    }

    adapter.Disconnect();
    return EXIT_SUCCESS;
}
