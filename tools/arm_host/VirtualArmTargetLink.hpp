#ifndef TOOLS_ARM_HOST_VIRTUAL_ARM_TARGET_LINK_HPP
#define TOOLS_ARM_HOST_VIRTUAL_ARM_TARGET_LINK_HPP

#include "can-lite/drivers/hub/VirtualCanHub.hpp"
#include "can-lite/drivers/implementation/PlatformCanAdapter.hpp"
#include "tools/arm_host/ArmTargetLink.hpp"
#include "tools/virtual_arm/VirtualArm.hpp"
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace arm_host
{
    class VirtualArmTargetLink
        : public ArmTargetLink
        , public VirtualMotors
    {
    public:
        static constexpr infra::Duration connectTimeout{ std::chrono::seconds{ 20 } };

        VirtualArmTargetLink(const ArmTarget& target, const arm::ArmConfig& config, uint32_t bitrate, const std::filesystem::path& folder);
        ~VirtualArmTargetLink() override;

        hal::Can& Can() override;
        bool Connected() override;
        void Poll() override;
        simulation::SimulatedArm* Simulation() override;
        infra::Duration ConnectTimeout() const override;
        std::optional<std::string> Failure() override;
        VirtualMotors* Motors() override;
        bool Adopt(const arm::ArmConfig& newConfig) override;
        uint32_t Bitrate() const override;

        VirtualMotorsStatus Status() const override;
        std::optional<std::string> Restart(std::size_t joint) override;
        void Silence(std::size_t joint, bool silent) override;
        void Collide(std::size_t joint, foc::NewtonMeter torque) override;
        std::optional<std::string> PowerOff(std::size_t joint) override;

    private:
        void StartMotors();
        void Collect();
        bool Started();

        arm::ArmConfig config;
        uint32_t bitrate;
        std::filesystem::path folder;
        std::array<bool, arm::jointCount> silenced{};
        arm::JointTorques torques{};
        services::VirtualCanHub hub;
        services::PlatformCanAdapter platformAdapter;
        bool joined{ false };
        bool up{ false };
        std::optional<std::string> failure;
        virtual_arm::VirtualArm virtualArm;
        std::mutex mutex;
        std::condition_variable closing;
        bool closed{ false };
        std::optional<std::optional<std::string>> startResult;
        std::jthread starter;
    };

    TargetOpener VirtualArmTargetOpener(TargetOpener other, uint32_t bitrate);
}

#endif
