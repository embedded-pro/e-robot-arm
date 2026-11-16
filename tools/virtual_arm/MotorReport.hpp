#ifndef TOOLS_VIRTUAL_ARM_MOTOR_REPORT_HPP
#define TOOLS_VIRTUAL_ARM_MOTOR_REPORT_HPP

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
}

#endif
