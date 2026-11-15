#ifndef TOOLS_VIRTUAL_ARM_PLANT_PORT_TEST_PLANT_PORT_BYTES_HPP
#define TOOLS_VIRTUAL_ARM_PLANT_PORT_TEST_PLANT_PORT_BYTES_HPP

#include "tools/virtual_arm/plant_port/PlantPortCodec.hpp"
#include <bit>
#include <cstdint>
#include <string>
#include <vector>

namespace virtual_arm::plant_port::test
{
    class Bytes
    {
    public:
        Bytes& U8(uint8_t value)
        {
            bytes.push_back(value);
            return *this;
        }

        Bytes& U16(uint16_t value)
        {
            return Little(value, 2);
        }

        Bytes& U32(uint32_t value)
        {
            return Little(value, 4);
        }

        Bytes& U64(uint64_t value)
        {
            return Little(value, 8);
        }

        Bytes& F32(float value)
        {
            return Little(std::bit_cast<uint32_t>(value), 4);
        }

        Bytes& Text(const std::string& text)
        {
            U8(static_cast<uint8_t>(text.size()));
            bytes.insert(bytes.end(), text.begin(), text.end());
            return *this;
        }

        std::vector<uint8_t> Frame(MessageType type) const
        {
            return EncodeFrame(type, bytes);
        }

        std::vector<uint8_t> bytes;

    private:
        Bytes& Little(uint64_t value, int size)
        {
            for (int index = 0; index != size; ++index)
                bytes.push_back(static_cast<uint8_t>(value >> (8 * index)));
            return *this;
        }
    };

    inline std::vector<uint8_t> HelloFrame(uint16_t version = protocolVersion)
    {
        return Bytes{}.U16(version).U8(20).Text("arm").Text("teknic").U32(20000).Frame(MessageType::hello);
    }

    inline std::vector<uint8_t> WindowFrame(float angle, float speed)
    {
        Bytes payload;
        payload.U32(7).U64(140).U8(10);
        for (std::size_t signal = 0; signal != 2 * signalCount; ++signal)
            payload.F32(0.0f);
        for (std::size_t signal = 0; signal != signalCount; ++signal)
            payload.F32(signal == rotorAngleSignal ? angle : signal == rotorSpeedSignal ? speed
                                                                                        : 0.0f);
        return payload.Frame(MessageType::window);
    }
}

#endif
