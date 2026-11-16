#include "tools/arm_host/VirtualArmTargetLink.hpp"
#include <cstdlib>
#include <format>
#include <thread>
#include <unistd.h>

namespace arm_host
{
    namespace
    {
        std::filesystem::path UserFolder()
        {
            for (const auto* variable : { "XDG_RUNTIME_DIR", "XDG_CACHE_HOME" })
                if (const auto* value = std::getenv(variable); value != nullptr && *value != '\0')
                    return value;
            if (const auto* home = std::getenv("HOME"); home != nullptr && *home != '\0')
                return std::filesystem::path{ home } / ".cache";
            return std::filesystem::current_path();
        }

        std::filesystem::path PrivateFolder()
        {
            const auto folder = UserFolder() / "e-robot-arm";
            std::error_code ignored;
            std::filesystem::create_directories(folder, ignored);
            std::filesystem::permissions(folder, std::filesystem::perms::owner_all, std::filesystem::perm_options::replace, ignored);
            return folder;
        }

        virtual_arm::VirtualArmOptions OptionsFor(const ArmTarget& target, const std::filesystem::path& folder)
        {
            virtual_arm::VirtualArmOptions options;
            options.engine = target.engine;
            options.workDirectory = folder;
            options.hubPort = services::loopback::virtualCanHubPort;
            options.hostBus = false;
            return options;
        }
    }

    VirtualArmTargetLink::VirtualArmTargetLink(const ArmTarget& target, const arm::ArmConfig& config, uint32_t bitrate, const std::filesystem::path& folder)
        : config(config)
        , folder(folder)
        , virtualArm(config, OptionsFor(target, folder))
    {
        if (!hub.Listen())
        {
            failure = std::format("the virtual CAN hub port {} is in use: another virtual arm or bus is running", services::loopback::virtualCanHubPort);
            return;
        }

        joined = platformAdapter.Adapter().Connect(std::string{ "virtual:" } + std::string{ virtual_arm::defaultBus }, bitrate);
        if (!joined)
        {
            failure = "the virtual CAN bus could not be joined";
            return;
        }

        starter = std::jthread{ [this]()
            {
                StartMotors();
            } };
    }

    VirtualArmTargetLink::~VirtualArmTargetLink()
    {
        while (starter.joinable() && !Started())
        {
            hub.Poll();
            std::this_thread::sleep_for(std::chrono::milliseconds{ 1 });
        }
        virtualArm.Stop();
        {
            std::scoped_lock lock{ mutex };
            closed = true;
        }
        closing.notify_all();
        starter = std::jthread{};
        if (joined)
            platformAdapter.Adapter().Disconnect();
    }

    void VirtualArmTargetLink::StartMotors()
    {
        auto result = virtualArm.Start();
        std::unique_lock lock{ mutex };
        startResult = std::move(result);
        closing.wait(lock, [this]()
            {
                return closed;
            });
    }

    bool VirtualArmTargetLink::Started()
    {
        std::scoped_lock lock{ mutex };
        return startResult.has_value();
    }

    hal::Can& VirtualArmTargetLink::Can()
    {
        return platformAdapter.Adapter();
    }

    bool VirtualArmTargetLink::Connected()
    {
        Collect();
        return up && platformAdapter.Adapter().IsConnected();
    }

    void VirtualArmTargetLink::Poll()
    {
        hub.Poll();
        platformAdapter.Adapter().ProcessReadEvent();
    }

    simulation::SimulatedArm* VirtualArmTargetLink::Simulation()
    {
        return nullptr;
    }

    infra::Duration VirtualArmTargetLink::ConnectTimeout() const
    {
        return connectTimeout;
    }

    std::optional<std::string> VirtualArmTargetLink::Failure()
    {
        Collect();
        return failure;
    }

    VirtualMotors* VirtualArmTargetLink::Motors()
    {
        return this;
    }

    bool VirtualArmTargetLink::Adopt(const arm::ArmConfig& newConfig)
    {
        if (virtualArm.Adopt(newConfig).has_value())
            return false;

        config = newConfig;
        return true;
    }

    VirtualMotorsStatus VirtualArmTargetLink::Status() const
    {
        return { virtualArm.Motors(), folder.string(), silenced, torques };
    }

    std::optional<std::string> VirtualArmTargetLink::Restart(std::size_t joint)
    {
        auto refusal = virtualArm.Restart(joint, joint < arm::jointCount ? config.motors[joint] : std::nullopt);
        if (!refusal.has_value())
            silenced[joint] = false;
        return refusal;
    }

    void VirtualArmTargetLink::Silence(std::size_t joint, bool silent)
    {
        if (joint >= arm::jointCount)
            return;

        virtualArm.Silence(joint, silent);
        silenced[joint] = silent;
    }

    void VirtualArmTargetLink::Collide(std::size_t joint, foc::NewtonMeter torque)
    {
        if (joint >= arm::jointCount)
            return;

        virtualArm.Collide(joint, torque);
        torques[joint] = torque;
    }

    std::optional<std::string> VirtualArmTargetLink::PowerOff(std::size_t joint)
    {
        auto refusal = virtualArm.PowerOff(joint);
        if (!refusal.has_value())
            silenced[joint] = false;
        return refusal;
    }

    void VirtualArmTargetLink::Collect()
    {
        if (up || failure.has_value())
            return;

        std::scoped_lock lock{ mutex };
        if (!startResult.has_value())
            return;

        failure = *startResult;
        up = !failure.has_value();
    }

    TargetOpener VirtualArmTargetOpener(TargetOpener other, uint32_t bitrate)
    {
        auto runs = std::make_shared<uint32_t>(0);
        return [other = std::move(other), bitrate, runs](const ArmTarget& target, const arm::ArmConfig& config) -> std::unique_ptr<ArmTargetLink>
        {
            if (target.kind != TargetKind::virtualArm)
                return other(target, config);

            const auto folder = PrivateFolder() / std::format("virtual-arm-{}-{}", getpid(), ++*runs);
            return std::make_unique<VirtualArmTargetLink>(target, config, bitrate, folder);
        };
    }
}
