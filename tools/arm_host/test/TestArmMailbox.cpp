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
    ASSERT_EQ(4u, events.size());
    EXPECT_EQ(arm_host::ArmEventKind::eventsDropped, events.front().kind);
    EXPECT_EQ(2u, events[1].command);
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

TEST_F(ArmMailboxTest, a_slow_consumer_learns_how_many_events_were_lost_and_where)
{
    for (uint32_t id = 1; id != 6; ++id)
        mailbox.OnEvent(Event(id));

    const auto events = mailbox.TakeEvents();

    ASSERT_EQ(4u, events.size());
    EXPECT_EQ(arm_host::ArmEventKind::eventsDropped, events.front().kind);
    EXPECT_EQ(2u, events.front().dropped);
    EXPECT_EQ(3u, events[1].command);
    EXPECT_EQ(2u, mailbox.DroppedEvents());

    mailbox.OnEvent(Event(6));
    const auto next = mailbox.TakeEvents();
    ASSERT_EQ(1u, next.size());
    EXPECT_EQ(6u, next.front().command);
}

TEST_F(ArmMailboxTest, a_slow_consumer_finds_the_trace_gap_on_the_first_sample_after_it)
{
    for (int sample = 0; sample != 5; ++sample)
        mailbox.OnTrace({ .time = infra::TimePoint{ std::chrono::milliseconds{ 10 * sample } } });

    const auto trace = mailbox.TakeTrace();

    ASSERT_EQ(2u, trace.size());
    EXPECT_EQ(3u, trace.front().droppedBefore);
    EXPECT_EQ(infra::TimePoint{ std::chrono::milliseconds{ 30 } }, trace.front().time);
    EXPECT_EQ(0u, trace.back().droppedBefore);
    EXPECT_EQ(3u, mailbox.DroppedTraceSamples());

    mailbox.OnTrace({});
    EXPECT_EQ(0u, mailbox.TakeTrace().front().droppedBefore);
}

TEST_F(ArmMailboxTest, a_consumer_that_keeps_up_sees_no_gap)
{
    for (int round = 0; round != 4; ++round)
    {
        mailbox.OnEvent(Event(1));
        mailbox.OnTrace({});
        for (const auto& event : mailbox.TakeEvents())
            EXPECT_NE(arm_host::ArmEventKind::eventsDropped, event.kind);
        for (const auto& sample : mailbox.TakeTrace())
            EXPECT_EQ(0u, sample.droppedBefore);
    }
}
