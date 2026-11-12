#ifndef INTEGRATION_TESTS_SUPPORT_VIRTUAL_CAN_BUS_HPP
#define INTEGRATION_TESTS_SUPPORT_VIRTUAL_CAN_BUS_HPP

#include "hal/interfaces/Can.hpp"
#include <deque>
#include <vector>

namespace integration
{
    class VirtualCanBus
    {
    public:
        class Port
            : public hal::Can
        {
        public:
            explicit Port(VirtualCanBus& bus);
            Port(const Port& other) = delete;
            Port& operator=(const Port& other) = delete;

            void SendData(Id id, const Message& data, const infra::Function<void(bool success)>& actionOnCompletion) override;
            void ReceiveData(const infra::Function<void(Id id, const Message& data)>& receivedAction) override;

            void Mute();

        private:
            friend class VirtualCanBus;

            VirtualCanBus& bus;
            infra::Function<void(Id id, const Message& data)> receive;
            bool muted{ false };
        };

    private:
        struct Frame
        {
            Port* from;
            hal::Can::Id id;
            hal::Can::Message data;
            infra::Function<void(bool success)> onDone;
        };

        void Send(Frame frame);
        void DeliverOldest();

        std::vector<Port*> ports;
        std::deque<Frame> frames;
    };
}

#endif
