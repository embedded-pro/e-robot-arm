#include "integration_tests/support/VirtualCanBus.hpp"
#include "infra/event/EventDispatcher.hpp"

namespace integration
{
    VirtualCanBus::Port::Port(VirtualCanBus& bus)
        : bus(bus)
    {
        bus.ports.push_back(this);
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

    void VirtualCanBus::Send(Frame frame)
    {
        frames.push_back(frame);
        infra::EventDispatcher::Instance().Schedule([this]()
            {
                DeliverOldest();
            });
    }

    void VirtualCanBus::DeliverOldest()
    {
        const auto frame = frames.front();
        frames.pop_front();

        if (!frame.from->muted)
            for (auto* port : ports)
                if (port != frame.from && !port->muted && port->receive)
                    port->receive(frame.id, frame.data);

        frame.onDone(true);
    }
}
