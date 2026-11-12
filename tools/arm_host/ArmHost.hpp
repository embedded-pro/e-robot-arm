#ifndef TOOLS_ARM_HOST_ARM_HOST_HPP
#define TOOLS_ARM_HOST_ARM_HOST_HPP

#include "hal/generic/TimerServiceGeneric.hpp"
#include "infra/event/EventDispatcherThreadAware.hpp"
#include "tools/arm_host/ArmCommands.hpp"
#include "tools/arm_host/ArmMailbox.hpp"
#include "tools/arm_host/ArmRuntime.hpp"
#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <variant>

namespace arm_host
{
    struct ArmRequest
    {
        uint32_t id;
        std::variant<ArmCommand, ArmTarget> what;
    };

    class ArmRequestQueue
    {
    public:
        void Push(ArmRequest request);
        std::optional<ArmRequest> Pop();

    private:
        std::mutex mutex;
        std::deque<ArmRequest> requests;
    };

    class ArmHost
        : public ArmCommands
    {
    public:
        static constexpr std::size_t eventCapacity{ 512 };
        static constexpr std::size_t traceCapacity{ 1024 };

        ArmHost(const arm::ArmConfig& config, const TargetOpener& opener, const ArmTarget& initial);
        ArmHost(const ArmHost& other) = delete;
        ArmHost& operator=(const ArmHost& other) = delete;
        ~ArmHost();

        uint32_t Post(const ArmCommand& command) override;
        uint32_t SwitchTarget(const ArmTarget& target);

        ArmSnapshot Snapshot() const override;
        std::vector<ArmEvent> TakeEvents();
        std::vector<TraceSample> TakeTrace();

    private:
        uint32_t Enqueue(std::variant<ArmCommand, ArmTarget> what);
        void RunOne();

        infra::EventDispatcherThreadAware::WithSize<256> dispatcher;
        hal::TimerServiceGeneric timerService;
        ArmMailbox mailbox{ eventCapacity, traceCapacity };
        std::optional<ArmRuntime> runtime;
        ArmRequestQueue requests;
        std::atomic<uint32_t> nextId{ 1 };
        std::atomic<bool> stopping{ false };
        std::jthread thread;
    };
}

#endif
