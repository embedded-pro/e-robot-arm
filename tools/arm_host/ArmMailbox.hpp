#ifndef TOOLS_ARM_HOST_ARM_MAILBOX_HPP
#define TOOLS_ARM_HOST_ARM_MAILBOX_HPP

#include "tools/arm_host/ArmSnapshot.hpp"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

namespace arm_host
{
    class ArmMailbox
        : public ArmSessionSink
    {
    public:
        ArmMailbox(std::size_t eventCapacity, std::size_t traceCapacity);

        void OnEvent(const ArmEvent& event) override;
        void OnSnapshot(const ArmSnapshot& snapshot) override;
        void OnTrace(const TraceSample& sample) override;

        ArmSnapshot Snapshot() const;
        std::vector<ArmEvent> TakeEvents();
        std::vector<TraceSample> TakeTrace();
        uint64_t DroppedEvents() const;
        uint64_t DroppedTraceSamples() const;

    private:
        template<class T>
        static void PushBounded(std::deque<T>& queue, const T& value, std::size_t capacity, uint64_t& dropped);

        mutable std::mutex mutex;
        std::size_t eventCapacity;
        std::size_t traceCapacity;
        ArmSnapshot snapshot;
        std::deque<ArmEvent> events;
        std::deque<TraceSample> trace;
        uint64_t droppedEvents{ 0 };
        uint64_t droppedTraceSamples{ 0 };
    };
}

#endif
