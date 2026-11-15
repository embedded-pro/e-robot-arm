#ifndef TOOLS_ARM_HOST_ARM_RUNTIME_HPP
#define TOOLS_ARM_HOST_ARM_RUNTIME_HPP

#include "tools/arm_host/ArmSession.hpp"
#include "tools/arm_host/ArmTargetLink.hpp"
#include <memory>
#include <optional>

namespace arm_host
{
    class ArmRuntime
    {
    public:
        static constexpr infra::Duration stopGrace{ std::chrono::milliseconds{ 100 } };
        static constexpr infra::Duration pollInterval{ std::chrono::milliseconds{ 1 } };
        static constexpr infra::Duration connectTimeout{ std::chrono::seconds{ 2 } };

        ArmRuntime(const arm::ArmProfile& profile, const TargetOpener& opener, ArmSessionSink& sink, const ArmTarget& initial);
        ArmRuntime(const ArmRuntime& other) = delete;
        ArmRuntime& operator=(const ArmRuntime& other) = delete;

        void Execute(uint32_t id, const ArmCommand& command);
        void SwitchTarget(uint32_t id, const ArmTarget& target);
        void LoadProfile(uint32_t id, const arm::ArmProfile& profile);

    private:
        struct Connecting
        {
            std::optional<uint32_t> announce;
            infra::TimePoint deadline;
        };

        bool Open(const ArmTarget& target, std::optional<uint32_t> announce);
        void ProceedConnecting();
        void Close();
        void FinishSwitch();
        void Report(ArmEventKind kind, uint32_t id, std::string_view reason = {});

        arm::ArmProfile profile;
        TargetOpener opener;
        ArmSessionSink& sink;
        ArmTarget current;
        std::unique_ptr<ArmTargetLink> link;
        std::optional<ArmSession> session;
        std::optional<Connecting> connecting;
        bool switching{ false };
        uint32_t switchId{ 0 };
        ArmTarget switchTarget;
        ArmTarget previousTarget;
        infra::TimerRepeating poll;
        infra::TimerSingleShot grace;
    };
}

#endif
