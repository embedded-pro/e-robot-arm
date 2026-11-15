#include "tools/virtual_arm/plant_port/PlantPortCodec.hpp"
#include <bit>

namespace virtual_arm::plant_port
{
    namespace
    {
        constexpr std::size_t headerSize{ 3 };

        template<class Integer>
        Integer Little(std::span<const uint8_t> bytes)
        {
            Integer value{ 0 };
            for (std::size_t index = 0; index != sizeof(Integer); ++index)
                value |= static_cast<Integer>(static_cast<Integer>(bytes[index]) << (8 * index));
            return value;
        }

        void PutU16(std::vector<uint8_t>& bytes, uint16_t value)
        {
            bytes.push_back(static_cast<uint8_t>(value & 0xff));
            bytes.push_back(static_cast<uint8_t>(value >> 8));
        }

        void PutF32(std::vector<uint8_t>& bytes, float value)
        {
            const auto raw = std::bit_cast<uint32_t>(value);
            for (std::size_t index = 0; index != 4; ++index)
                bytes.push_back(static_cast<uint8_t>(raw >> (8 * index)));
        }

        template<class Message, class Read>
        std::optional<Message> Decode(const Frame& frame, MessageType type, Read read)
        {
            if (frame.type != static_cast<uint8_t>(type))
                return std::nullopt;

            PayloadReader reader{ frame.payload };
            auto message = read(reader);
            if (!reader.Valid())
                return std::nullopt;
            return message;
        }
    }

    void FrameAssembler::Append(std::span<const uint8_t> bytes)
    {
        pending.insert(pending.end(), bytes.begin(), bytes.end());
    }

    std::optional<Frame> FrameAssembler::Next()
    {
        if (pending.size() < headerSize)
            return std::nullopt;

        const auto length = Little<uint16_t>(std::span{ pending }.subspan(1, 2));
        if (pending.size() < headerSize + length)
            return std::nullopt;

        Frame frame{ pending[0], { pending.begin() + headerSize, pending.begin() + headerSize + length } };
        pending.erase(pending.begin(), pending.begin() + headerSize + length);
        return frame;
    }

    PayloadReader::PayloadReader(std::span<const uint8_t> payload)
        : payload(payload)
    {}

    bool PayloadReader::Take(std::size_t size)
    {
        valid = valid && position + size <= payload.size();
        return valid;
    }

    uint8_t PayloadReader::U8()
    {
        return Take(1) ? payload[position++] : 0;
    }

    uint16_t PayloadReader::U16()
    {
        if (!Take(2))
            return 0;
        position += 2;
        return Little<uint16_t>(payload.subspan(position - 2, 2));
    }

    uint32_t PayloadReader::U32()
    {
        if (!Take(4))
            return 0;
        position += 4;
        return Little<uint32_t>(payload.subspan(position - 4, 4));
    }

    uint64_t PayloadReader::U64()
    {
        if (!Take(8))
            return 0;
        position += 8;
        return Little<uint64_t>(payload.subspan(position - 8, 8));
    }

    float PayloadReader::F32()
    {
        return std::bit_cast<float>(U32());
    }

    std::string PayloadReader::Text()
    {
        const auto length = U8();
        if (!Take(length))
            return {};
        std::string text(payload.begin() + static_cast<std::ptrdiff_t>(position), payload.begin() + static_cast<std::ptrdiff_t>(position + length));
        position += length;
        return text;
    }

    bool PayloadReader::Valid() const
    {
        return valid;
    }

    std::optional<Hello> DecodeHello(const Frame& frame)
    {
        return Decode<Hello>(frame, MessageType::hello, [](PayloadReader& reader)
            {
                Hello hello;
                hello.version = reader.U16();
                hello.node = reader.U8();
                hello.bus = reader.Text();
                hello.plant = reader.Text();
                hello.controlRateHz = reader.U32();
                return hello;
            });
    }

    std::optional<Status> DecodeStatus(const Frame& frame)
    {
        return Decode<Status>(frame, MessageType::status, [](PayloadReader& reader)
            {
                Status status;
                status.controlPeriods = reader.U64();
                status.realTimeFactor = reader.F32();
                status.overruns = reader.U32();
                status.paused = reader.U8() != 0;
                status.leds = reader.U8();
                status.protection = reader.U8();
                return status;
            });
    }

    std::optional<Refused> DecodeRefused(const Frame& frame)
    {
        return Decode<Refused>(frame, MessageType::refused, [](PayloadReader& reader)
            {
                Refused refused;
                refused.request = reader.U8();
                refused.reason = reader.U8();
                return refused;
            });
    }

    std::optional<std::string> DecodeLogLine(const Frame& frame)
    {
        return Decode<std::string>(frame, MessageType::logLine, [](PayloadReader& reader)
            {
                return reader.Text();
            });
    }

    std::optional<Window> DecodeWindow(const Frame& frame)
    {
        return Decode<Window>(frame, MessageType::window, [](PayloadReader& reader)
            {
                Window window;
                window.sequence = reader.U32();
                window.firstPeriod = reader.U64();
                window.periods = reader.U8();
                for (auto* signals : { &window.minimum, &window.maximum, &window.last })
                    for (auto& value : *signals)
                        value = reader.F32();
                return window;
            });
    }

    std::vector<uint8_t> EncodeFrame(MessageType type, std::span<const uint8_t> payload)
    {
        std::vector<uint8_t> bytes{ static_cast<uint8_t>(type) };
        PutU16(bytes, static_cast<uint16_t>(payload.size()));
        bytes.insert(bytes.end(), payload.begin(), payload.end());
        return bytes;
    }

    std::vector<uint8_t> EncodeConstantExternalTorque(float newtonMeter)
    {
        std::vector<uint8_t> payload{ static_cast<uint8_t>(TorqueForm::constant) };
        PutF32(payload, newtonMeter);
        for (int unused = 0; unused != 3; ++unused)
            PutF32(payload, 0.0f);
        return EncodeFrame(MessageType::externalTorque, payload);
    }

    std::vector<uint8_t> EncodeStream(bool on)
    {
        const std::array<uint8_t, 1> payload{ static_cast<uint8_t>(on ? 1 : 0) };
        return EncodeFrame(MessageType::stream, payload);
    }
}
