#include "tools/arm_host/ArmMailbox.hpp"
#include "gmock/gmock.h"

namespace
{
    class ArmMailboxTest
        : public testing::Test
    {
    public:
        arm_host::ArmEvent Event(uint32_t id) const
        {
            return { .kind = arm_host::ArmEventKind::accepted, .command = id };
        }

        arm_host::ArmMailbox mailbox{ 3, 2 };
    };
}

TEST_F(ArmMailboxTest, numbers_every_snapshot_it_receives)
{
    mailbox.OnSnapshot({ .targetReady = true });
    mailbox.OnSnapshot({ .targetReady = true });

    EXPECT_EQ(2u, mailbox.Snapshot().sequence);
    EXPECT_TRUE(mailbox.Snapshot().targetReady);
}

TEST_F(ArmMailboxTest, hands_out_events_in_order_once)
{
    mailbox.OnEvent(Event(1));
    mailbox.OnEvent(Event(2));

    const auto events = mailbox.TakeEvents();
    ASSERT_EQ(2u, events.size());
    EXPECT_EQ(1u, events[0].command);
    EXPECT_EQ(2u, events[1].command);
    EXPECT_TRUE(mailbox.TakeEvents().empty());
}

TEST_F(ArmMailboxTest, drops_the_oldest_event_when_full_and_counts_it)
{
    for (uint32_t id = 1; id != 5; ++id)
        mailbox.OnEvent(Event(id));

    const auto events = mailbox.TakeEvents();
    ASSERT_EQ(3u, events.size());
    EXPECT_EQ(2u, events.front().command);
    EXPECT_EQ(1u, mailbox.DroppedEvents());
}

TEST_F(ArmMailboxTest, drops_the_oldest_trace_sample_when_full_and_counts_it)
{
    for (int sample = 0; sample != 3; ++sample)
        mailbox.OnTrace({});

    EXPECT_EQ(2u, mailbox.TakeTrace().size());
    EXPECT_EQ(1u, mailbox.DroppedTraceSamples());
    EXPECT_TRUE(mailbox.TakeTrace().empty());
}
