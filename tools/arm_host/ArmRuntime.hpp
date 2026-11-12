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

        ArmRuntime(const arm::ArmConfig& config, const TargetOpener& opener, ArmSessionSink& sink, const ArmTarget& initial);
        ArmRuntime(const ArmRuntime& other) = delete;
        ArmRuntime& operator=(const ArmRuntime& other) = delete;

        void Execute(uint32_t id, const ArmCommand& command);
        void SwitchTarget(uint32_t id, const ArmTarget& target);

    private:
        bool Open(const ArmTarget& target);
        void Close();
        void FinishSwitch();
        void Report(ArmEventKind kind, uint32_t id, std::string_view reason = {});

        arm::ArmConfig config;
        TargetOpener opener;
        ArmSessionSink& sink;
        ArmTarget current;
        std::unique_ptr<ArmTargetLink> link;
        std::optional<ArmSession> session;
        bool switching{ false };
        uint32_t switchId{ 0 };
        ArmTarget switchTarget;
        ArmTarget previousTarget;
        infra::TimerRepeating poll;
        infra::TimerSingleShot grace;
    };
}

#endif
