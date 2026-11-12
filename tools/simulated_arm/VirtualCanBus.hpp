#ifndef TOOLS_SIMULATED_ARM_VIRTUAL_CAN_BUS_HPP
#define TOOLS_SIMULATED_ARM_VIRTUAL_CAN_BUS_HPP

#include "hal/interfaces/Can.hpp"
#include <deque>
#include <memory>
#include <vector>

namespace simulation
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
            ~Port();

            void SendData(Id id, const Message& data, const infra::Function<void(bool success)>& actionOnCompletion) override;
            void ReceiveData(const infra::Function<void(Id id, const Message& data)>& receivedAction) override;

            void Mute();

        private:
            friend class VirtualCanBus;

            VirtualCanBus& bus;
            infra::Function<void(Id id, const Message& data)> receive;
            bool muted{ false };
        };

        VirtualCanBus() = default;
        VirtualCanBus(const VirtualCanBus& other) = delete;
        VirtualCanBus& operator=(const VirtualCanBus& other) = delete;

        bool Idle() const;

    private:
        struct Frame
        {
            Port* from;
            hal::Can::Id id;
            hal::Can::Message data;
            infra::Function<void(bool success)> onDone;
        };

        void Send(const Frame& frame);
        void DeliverOldest();
        void Detach(const Port& port);

        std::vector<Port*> ports;
        std::deque<Frame> frames;
        std::shared_ptr<VirtualCanBus*> self{ std::make_shared<VirtualCanBus*>(this) };
    };
}

#endif
