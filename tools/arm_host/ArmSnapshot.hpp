#ifndef TOOLS_ARM_HOST_ARM_SNAPSHOT_HPP
#define TOOLS_ARM_HOST_ARM_SNAPSHOT_HPP

#include "core/arm/ArmConfig.hpp"
#include "core/arm_model/ArmModel.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "infra/timer/Timer.hpp"
#include "tools/arm_host/ArmEvent.hpp"
#include "tools/arm_host/ArmTarget.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace arm_host
{
    struct ArmSnapshot
    {
        uint64_t sequence{ 0 };
        ArmTarget target{};
        bool targetReady{ false };
        std::array<arm::JointReading, arm::jointCount> readings{};
        std::optional<arm::JointPositions> commanded{};
        can::ContractCompatibility contracts{ can::ContractCompatibility::unknown };
        bool tripped{ false };
        bool moving{ false };
        arm::ArmState state{ arm::ArmState::unpowered };
        std::optional<arm::Misconfiguration> misconfiguration{};
        uint32_t overruns{ 0 };
        arm::Placement toolPose{};
        arm::Singularities singularities{};
        std::string profileName{};
        arm::ArmConfig config{};
    };

    struct TraceSample
    {
        infra::TimePoint time{};
        arm::JointPositions commanded{};
        arm::JointPositions measured{};
    };

    class ArmSessionSink
    {
    public:
        virtual void OnEvent(const ArmEvent& event) = 0;
        virtual void OnSnapshot(const ArmSnapshot& snapshot) = 0;
        virtual void OnTrace(const TraceSample& sample) = 0;

    protected:
        ~ArmSessionSink() = default;
    };
}

#endif
