#ifndef TOOLS_ARM_HOST_ARM_HOST_HPP
#define TOOLS_ARM_HOST_ARM_HOST_HPP

#include "hal/generic/TimerServiceGeneric.hpp"
#include "infra/event/EventDispatcherThreadAware.hpp"
#include "tools/arm_host/ArmCommands.hpp"
#include "tools/arm_host/ArmMailbox.hpp"
#include "tools/arm_host/ArmRuntime.hpp"
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <variant>
#include <vector>

namespace arm_host
{
    struct CommissioningRequest
    {
        bool on;
    };

    using ArmRequestKind = std::variant<ArmCommand, ArmTarget, arm::ArmProfile, CommissioningRequest>;

    struct ArmRequest
    {
        uint32_t id;
        ArmRequestKind what;
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
        static constexpr std::chrono::milliseconds detectTimeout{ 500 };

        ArmHost(const arm::ArmProfile& profile, const TargetOpener& opener, const ArmTarget& initial);
        ArmHost(const ArmHost& other) = delete;
        ArmHost& operator=(const ArmHost& other) = delete;
        ~ArmHost();

        uint32_t Post(const ArmCommand& command) override;
        uint32_t SwitchTarget(const ArmTarget& target) override;
        uint32_t LoadProfile(const arm::ArmProfile& profile) override;
        uint32_t SetCommissioning(bool on) override;
        std::vector<std::string> DetectAdapters(const std::function<std::vector<std::string>()>& detect);

        ArmSnapshot Snapshot() const override;
        std::vector<ArmEvent> TakeEvents();
        std::vector<TraceSample> TakeTrace();

    private:
        uint32_t Enqueue(ArmRequestKind what);
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
