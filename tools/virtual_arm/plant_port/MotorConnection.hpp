#pragma once

#include "tools/virtual_arm/plant_port/PlantPortCodec.hpp"
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace virtual_arm::plant_port
{
    inline constexpr std::size_t keptLogLines{ 20 };

    class MotorConnection
    {
    public:
        void Receive(std::span<const uint8_t> bytes);

        const std::optional<Hello>& Introduction() const;
        const std::optional<std::string>& Failure() const;
        std::optional<float> RotorAngle() const;
        std::optional<float> RotorSpeed() const;
        std::optional<float> ElectricalTorque() const;
        std::optional<uint64_t> LastPeriod() const;
        bool Paused() const;
        const std::optional<Status>& LastStatus() const;
        const std::optional<Refused>& LastRefusal() const;
        const std::vector<std::string>& RecentLog() const;

        void Stream(bool on);
        void Pause();
        void Resume();
        void ShaftSpeed(const std::optional<ShaftDrive>& drive);
        void ClearAll();
        std::vector<uint8_t> TakeOutgoing();

    private:
        void Handle(const Frame& frame);
        void Introduce(const Hello& hello);
        void Follow(const Window& window);
        void Fail(std::string reason);
        void Queue(const std::vector<uint8_t>& frame);

        FrameAssembler assembler;
        std::optional<Hello> introduction;
        std::optional<std::string> failure;
        std::optional<float> rawAngle;
        float unwrappedAngle{ 0.0f };
        std::optional<float> speed;
        std::optional<float> torque;
        std::optional<uint64_t> lastPeriod;
        bool paused{ false };
        std::optional<Status> status;
        std::optional<Refused> refusal;
        std::vector<std::string> log;
        std::vector<uint8_t> outgoing;
    };
}
