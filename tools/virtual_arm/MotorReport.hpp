#pragma once

#include "core/joint_bus/interfaces/JointBus.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace virtual_arm
{
    enum class MotorPhase : uint8_t
    {
        stopped,
        starting,
        up,
        ended
    };

    struct MotorReport
    {
        MotorPhase phase{ MotorPhase::stopped };
        std::string plant;
        std::optional<int> exitCode;

        friend bool operator==(const MotorReport& lhs, const MotorReport& rhs) = default;
    };

    using MotorReports = std::array<MotorReport, arm::jointCount>;

    struct MotorTiming
    {
        float realTimeFactor{ 0.0f };
        uint32_t overruns{ 0 };
    };

    using MotorTimings = std::array<std::optional<MotorTiming>, arm::jointCount>;
}
