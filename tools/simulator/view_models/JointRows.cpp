#include "tools/simulator/view_models/JointRows.hpp"
#include <algorithm>
#include <format>

namespace simulator
{
    namespace
    {
        constexpr std::string_view placeholder{ "—" };

        std::string Fixed(float value, int decimals, std::string_view unit)
        {
            return std::format("{:.{}f} {}", value, decimals, unit);
        }

        float LimitFraction(foc::Radians position, const arm::JointLimit& limit)
        {
            const auto span = limit.upper.Value() - limit.lower.Value();
            if (span <= 0.0f)
                return 0.5f;
            return std::clamp((position.Value() - limit.lower.Value()) / span, 0.0f, 1.0f);
        }
    }

    std::string_view StateName(can::FocMotorState state)
    {
        static constexpr std::array<std::string_view, 6> names{ "Idle", "Running", "Fault", "Calibrating", "Partly calibrated", "Ready" };
        const auto index = static_cast<std::size_t>(state);
        return index < names.size() ? names[index] : "Unknown";
    }

    std::string_view FaultName(can::FocFaultCode fault)
    {
        static constexpr std::array<std::string_view, 14> names{ "—", "Overcurrent", "Overvoltage", "Undervoltage", "Overtemperature", "Sensor fault",
            "Stall", "Phase missing", "Hardware fault", "Calibration failed", "Overspeed", "Following error", "Command timeout", "Position limit" };
        const auto index = static_cast<std::size_t>(fault);
        return index < names.size() ? names[index] : "Unknown";
    }

    ui::theme::StatusLevel LevelOf(const arm::JointReading& reading)
    {
        switch (reading.state)
        {
            case can::FocMotorState::running:
                return ui::theme::StatusLevel::Ok;
            case can::FocMotorState::fault:
                return ui::theme::StatusLevel::Fault;
            case can::FocMotorState::calibrating:
            case can::FocMotorState::partialCalibration:
                return ui::theme::StatusLevel::Warning;
            default:
                return ui::theme::StatusLevel::Neutral;
        }
    }

    std::array<JointRow, arm::jointCount> JointRows(const arm_host::ArmSnapshot& snapshot, const arm::JointLimits& limits)
    {
        std::array<JointRow, arm::jointCount> rows{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            auto& row = rows[joint];
            row.number = std::to_string(joint + 1);
            if (!snapshot.targetReady)
            {
                row.position = placeholder;
                row.speed = placeholder;
                row.state = placeholder;
                row.fault = placeholder;
                continue;
            }

            const auto& reading = snapshot.readings[joint];
            row.position = Fixed(reading.position.Value(), 3, "rad");
            row.speed = Fixed(reading.speed.Value(), 2, "rad/s");
            row.state = StateName(reading.state);
            row.fault = FaultName(reading.fault);
            row.level = LevelOf(reading);
            row.limitFraction = LimitFraction(reading.position, limits[joint]);
        }
        return rows;
    }
}
