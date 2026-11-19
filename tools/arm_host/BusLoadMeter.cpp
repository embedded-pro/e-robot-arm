#include "tools/arm_host/BusLoadMeter.hpp"
#include <chrono>

namespace arm_host
{
    BusLoadMeter::BusLoadMeter(hal::Can& can, uint32_t bitrate)
        : can(can)
        , bitrate(bitrate)
        , window(std::chrono::seconds{ 1 }, [this]()
              {
                  CloseWindow();
              })
    {
        can.ReceiveData([this](Id id, const Message& data)
            {
                Count(data);
                if (received)
                    received(id, data);
            });
    }

    void BusLoadMeter::SendData(Id id, const Message& data, const infra::Function<void(bool success)>& actionOnCompletion)
    {
        Count(data);
        can.SendData(id, data, actionOnCompletion);
    }

    void BusLoadMeter::ReceiveData(const infra::Function<void(Id id, const Message& data)>& receivedAction)
    {
        received = receivedAction;
    }

    std::optional<float> BusLoadMeter::Load() const
    {
        return load;
    }

    void BusLoadMeter::Count(const Message& data)
    {
        bits += frameOverheadBits + 8u * data.size();
    }

    void BusLoadMeter::CloseWindow()
    {
        load = static_cast<float>(static_cast<double>(bits) / static_cast<double>(bitrate));
        bits = 0;
    }
}
