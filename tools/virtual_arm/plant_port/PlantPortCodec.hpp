#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace virtual_arm::plant_port
{
    inline constexpr uint16_t protocolVersion{ 3 };
    inline constexpr std::size_t signalCount{ 25 };
    inline constexpr std::size_t rotorAngleSignal{ 16 };
    inline constexpr std::size_t rotorSpeedSignal{ 18 };
    inline constexpr std::size_t electricalTorqueSignal{ 19 };

    enum class MessageType : uint8_t
    {
        hello = 0x01,
        status = 0x02,
        changes = 0x03,
        refused = 0x04,
        logLine = 0x05,
        window = 0x10,
        pause = 0x81,
        resume = 0x82,
        clearAll = 0x88,
        shaftSpeed = 0x89,
        stream = 0x90
    };

    struct Frame
    {
        uint8_t type{ 0 };
        std::vector<uint8_t> payload;
    };

    class FrameAssembler
    {
    public:
        void Append(std::span<const uint8_t> bytes);
        std::optional<Frame> Next();

    private:
        std::vector<uint8_t> pending;
    };

    class PayloadReader
    {
    public:
        explicit PayloadReader(std::span<const uint8_t> payload);

        uint8_t U8();
        uint16_t U16();
        uint32_t U32();
        uint64_t U64();
        float F32();
        std::string Text();
        bool Valid() const;

    private:
        bool Take(std::size_t size);

        std::span<const uint8_t> payload;
        std::size_t position{ 0 };
        bool valid{ true };
    };

    struct Hello
    {
        uint16_t version{ 0 };
        uint8_t node{ 0 };
        std::string bus;
        std::string plant;
        uint32_t controlRateHz{ 0 };
    };

    struct Status
    {
        uint64_t controlPeriods{ 0 };
        float realTimeFactor{ 0.0f };
        uint32_t overruns{ 0 };
        bool paused{ false };
        uint8_t leds{ 0 };
        uint8_t protection{ 0 };
    };

    struct Refused
    {
        uint8_t request{ 0 };
        uint8_t reason{ 0 };
    };

    struct Window
    {
        uint32_t sequence{ 0 };
        uint64_t firstPeriod{ 0 };
        uint8_t periods{ 0 };
        std::array<float, signalCount> minimum{};
        std::array<float, signalCount> maximum{};
        std::array<float, signalCount> last{};
    };

    struct ShaftDrive
    {
        float speed{ 0.0f };
        uint64_t period{ 0 };
        float torque{ 0.0f };
        float inertia{ 0.0f };
    };

    std::optional<Hello> DecodeHello(const Frame& frame);
    std::optional<Status> DecodeStatus(const Frame& frame);
    std::optional<Refused> DecodeRefused(const Frame& frame);
    std::optional<std::string> DecodeLogLine(const Frame& frame);
    std::optional<Window> DecodeWindow(const Frame& frame);

    std::vector<uint8_t> EncodeFrame(MessageType type, std::span<const uint8_t> payload = {});
    std::vector<uint8_t> EncodeShaftSpeed(const std::optional<ShaftDrive>& drive);
    std::vector<uint8_t> EncodeStream(bool on);
}
