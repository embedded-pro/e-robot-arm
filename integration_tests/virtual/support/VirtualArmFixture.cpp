#include "integration_tests/virtual/support/VirtualArmFixture.hpp"
#include "tools/arm_host/AdapterTargetLink.hpp"
#include "tools/arm_host/DefaultArmConfig.hpp"
#include "tools/arm_host/ProfileFile.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <thread>

namespace integration
{
    namespace
    {
        constexpr auto pollPeriod{ std::chrono::milliseconds{ 20 } };
        constexpr uint32_t bitrate{ 1000000 };
        int scenariosStarted{ 0 };

        std::filesystem::path Engine()
        {
            const auto* engine = std::getenv("E_FOC_VIRTUAL_MOTOR");
            return engine != nullptr ? std::filesystem::path{ engine } : std::filesystem::path{ "e-foc-virtual-motor" };
        }
    }

    VirtualArmFixture::VirtualArmFixture()
    {
        auto read = arm_host::ReadProfileFile(VIRTUAL_ARM_PROFILE);
        if (const auto* profileRead = std::get_if<arm::ArmProfile>(&read))
            profile = *profileRead;
    }

    std::optional<std::string> VirtualArmFixture::Start()
    {
        virtual_arm::VirtualArmOptions options;
        options.engine = Engine();
        options.workDirectory = std::filesystem::temp_directory_path() / "e-robot-arm-virtual-scenarios" / std::format("scenario{}", ++scenariosStarted);
        virtualArm.emplace(profile.config, options);
        if (auto problem = virtualArm->Start(); problem.has_value())
            return problem;

        host.emplace(profile, arm_host::HostTargetOpener(arm_host::simulatedTimeConstant, bitrate),
            arm_host::ArmTarget{ arm_host::TargetKind::virtualBus, std::string{ "virtual:" } + options.bus });
        if (!WaitFor([](const arm_host::ArmSnapshot& snapshot)
                {
                    return snapshot.targetReady;
                },
                std::chrono::seconds{ 10 }))
            return std::string{ "the arm did not join the virtual bus" };
        return std::nullopt;
    }

    bool VirtualArmFixture::Command(const arm_host::ArmCommand& command)
    {
        const auto id = host->Post(command);
        bool accepted{ false };
        WaitForEvent([id, &accepted](const arm_host::ArmEvent& event)
            {
                accepted = event.kind == arm_host::ArmEventKind::accepted;
                return event.command == id && (accepted || event.kind == arm_host::ArmEventKind::refused);
            },
            std::chrono::seconds{ 5 });
        return accepted;
    }

    bool VirtualArmFixture::WaitFor(const std::function<bool(const arm_host::ArmSnapshot&)>& condition, std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!condition(host->Snapshot()))
        {
            if (std::chrono::steady_clock::now() >= deadline)
                return false;
            Collect();
            std::this_thread::sleep_for(pollPeriod);
        }
        Collect();
        return true;
    }

    bool VirtualArmFixture::WaitForEvent(const std::function<bool(const arm_host::ArmEvent&)>& condition, std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::size_t checked{ 0 };
        while (true)
        {
            Collect();
            for (; checked != events.size(); ++checked)
                if (condition(events[checked]))
                    return true;
            if (std::chrono::steady_clock::now() >= deadline)
                return false;
            std::this_thread::sleep_for(pollPeriod);
        }
    }

    void VirtualArmFixture::ForgetTrace()
    {
        Collect();
        trace.clear();
    }

    float VirtualArmFixture::LargestTrackingError()
    {
        Collect();
        float largest{ 0.0f };
        for (const auto& sample : trace)
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                largest = std::max(largest, std::abs(sample.commanded[joint].Value() - sample.measured[joint].Value()));
        return largest;
    }

    std::string VirtualArmFixture::History()
    {
        Collect();
        const auto snapshot = host->Snapshot();
        auto history = std::format("arm state {}, tripped {}; events:\n", arm_host::StateName(snapshot.state), snapshot.tripped);
        for (const auto& event : events)
            history += std::format("  kind {} command {} '{}' reason '{}' joint {} fault {} state {}\n", static_cast<int>(event.kind), event.command, event.commandName,
                event.reason, event.joint == arm_host::noJoint ? -1 : static_cast<int>(event.joint), static_cast<int>(event.fault), arm_host::StateName(event.state));
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            history += std::format("  J{} state {} fault {} silent {}\n", joint + 1, static_cast<int>(snapshot.readings[joint].state),
                static_cast<int>(snapshot.readings[joint].fault), snapshot.readings[joint].silent);
        const auto timings = virtualArm->Timings();
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (timings[joint].has_value())
                history += std::format("  motor {} real-time factor {:.3f} overruns {}\n", joint + 1, timings[joint]->realTimeFactor, timings[joint]->overruns);
        return history;
    }

    void VirtualArmFixture::Collect()
    {
        auto newEvents = host->TakeEvents();
        events.insert(events.end(), newEvents.begin(), newEvents.end());
        auto newTrace = host->TakeTrace();
        trace.insert(trace.end(), newTrace.begin(), newTrace.end());
    }
}
