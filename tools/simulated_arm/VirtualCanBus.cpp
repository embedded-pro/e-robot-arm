#include "tools/simulated_arm/VirtualCanBus.hpp"
#include "infra/event/EventDispatcher.hpp"
#include <algorithm>

namespace simulation
{
    VirtualCanBus::Port::Port(VirtualCanBus& bus)
        : bus(bus)
    {
        bus.ports.push_back(this);
    }

    VirtualCanBus::Port::~Port()
    {
        bus.Detach(*this);
    }

    void VirtualCanBus::Port::SendData(Id id, const Message& data, const infra::Function<void(bool success)>& actionOnCompletion)
    {
        bus.Send({ this, id, data, actionOnCompletion });
    }

    void VirtualCanBus::Port::ReceiveData(const infra::Function<void(Id id, const Message& data)>& receivedAction)
    {
        receive = receivedAction;
    }

    void VirtualCanBus::Port::Mute()
    {
        muted = true;
    }

    bool VirtualCanBus::Idle() const
    {
        return frames.empty();
    }

    void VirtualCanBus::Send(const Frame& frame)
    {
        frames.push_back(frame);
        infra::EventDispatcher::Instance().Schedule([bus = std::weak_ptr<VirtualCanBus*>{ self }]()
            {
                if (const auto alive = bus.lock())
                    (*alive)->DeliverOldest();
            });
    }

    void VirtualCanBus::DeliverOldest()
    {
        if (frames.empty())
            return;

        const auto frame = frames.front();
        frames.pop_front();

        if (!frame.from->muted)
            for (const auto* port : ports)
                if (port != frame.from && !port->muted && port->receive)
                    port->receive(frame.id, frame.data);

        frame.onDone(true);
    }

    void VirtualCanBus::Detach(const Port& port)
    {
        std::erase(ports, &port);
        std::erase_if(frames, [&port](const Frame& frame)
            {
                return frame.from == &port;
            });
    }
}
