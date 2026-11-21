#pragma once

#include "hal/interfaces/Can.hpp"
#include "infra/timer/Timer.hpp"
#include <cstdint>
#include <optional>

namespace arm_host
{
    class BusLoadMeter
        : public hal::Can
    {
    public:
        static constexpr uint32_t frameOverheadBits{ 67 };
        static constexpr uint32_t defaultBitrate{ 1000000 };

        BusLoadMeter(hal::Can& can, uint32_t bitrate);

        void SendData(Id id, const Message& data, const infra::Function<void(bool success)>& actionOnCompletion) override;
        void ReceiveData(const infra::Function<void(Id id, const Message& data)>& receivedAction) override;

        std::optional<float> Load() const;

    private:
        void Count(const Message& data);
        void CloseWindow();

        hal::Can& can;
        uint32_t bitrate;
        uint64_t bits{ 0 };
        std::optional<float> load;
        infra::Function<void(Id id, const Message& data)> received;
        infra::TimerRepeating window;
    };
}
