#include "tools/arm_host/ArmMailbox.hpp"

namespace arm_host
{
    ArmMailbox::ArmMailbox(std::size_t eventCapacity, std::size_t traceCapacity)
        : eventCapacity(eventCapacity)
        , traceCapacity(traceCapacity)
    {}

    void ArmMailbox::OnEvent(const ArmEvent& event)
    {
        std::scoped_lock lock{ mutex };
        PushBounded(events, event, eventCapacity, droppedEvents);
    }

    void ArmMailbox::OnSnapshot(const ArmSnapshot& newSnapshot)
    {
        std::scoped_lock lock{ mutex };
        const auto sequence = snapshot.sequence + 1;
        snapshot = newSnapshot;
        snapshot.sequence = sequence;
    }

    void ArmMailbox::OnTrace(const TraceSample& sample)
    {
        std::scoped_lock lock{ mutex };
        PushBounded(trace, sample, traceCapacity, droppedTraceSamples);
    }

    ArmSnapshot ArmMailbox::Snapshot() const
    {
        std::scoped_lock lock{ mutex };
        return snapshot;
    }

    std::vector<ArmEvent> ArmMailbox::TakeEvents()
    {
        std::scoped_lock lock{ mutex };
        std::vector<ArmEvent> taken{ events.begin(), events.end() };
        events.clear();
        return taken;
    }

    std::vector<TraceSample> ArmMailbox::TakeTrace()
    {
        std::scoped_lock lock{ mutex };
        std::vector<TraceSample> taken{ trace.begin(), trace.end() };
        trace.clear();
        return taken;
    }

    uint64_t ArmMailbox::DroppedEvents() const
    {
        std::scoped_lock lock{ mutex };
        return droppedEvents;
    }

    uint64_t ArmMailbox::DroppedTraceSamples() const
    {
        std::scoped_lock lock{ mutex };
        return droppedTraceSamples;
    }

    template<class T>
    void ArmMailbox::PushBounded(std::deque<T>& queue, const T& value, std::size_t capacity, uint64_t& dropped)
    {
        if (queue.size() == capacity)
        {
            queue.pop_front();
            ++dropped;
        }
        queue.push_back(value);
    }
}
