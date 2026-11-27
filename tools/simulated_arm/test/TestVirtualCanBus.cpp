#include "infra/event/EventDispatcher.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "tools/simulated_arm/VirtualCanBus.hpp"
#include "gtest/gtest.h"
#include <cstdint>

namespace
{
    class VirtualCanBusTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        VirtualCanBusTest()
        {
            node.ReceiveData([this](hal::Can::Id id, const hal::Can::Message& data)
                {
                    node.SendData(id, data, [](bool) {});
                });
            host.ReceiveData([this](hal::Can::Id, const hal::Can::Message&)
                {
                    ++answers;
                });
        }

        void SendRequests(uint32_t remaining)
        {
            if (remaining == 0)
                return;

            host.SendData(hal::Can::Id::Create29BitId(remaining), hal::Can::Message{}, [this, remaining](bool)
                {
                    SendRequests(remaining - 1);
                });
        }

        simulation::VirtualCanBus bus;
        simulation::VirtualCanBus::Port host{ bus };
        simulation::VirtualCanBus::Port node{ bus };
        uint32_t answers{ 0 };
    };
}

TEST_F(VirtualCanBusTest, requests_sent_one_after_another_are_all_answered_before_the_next_scheduled_action)
{
    SendRequests(6);
    uint32_t answersSeen{ 0 };
    infra::EventDispatcher::Instance().Schedule([this, &answersSeen]()
        {
            answersSeen = answers;
        });

    ExecuteAllActions();

    EXPECT_EQ(6u, answersSeen);
    EXPECT_TRUE(bus.Idle());
}
