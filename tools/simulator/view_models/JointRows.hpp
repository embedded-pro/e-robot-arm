#pragma once

#include "tools/arm_host/ArmSnapshot.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <string>
#include <string_view>

namespace simulator
{
    struct JointRow
    {
        std::string number;
        std::string position;
        std::string speed;
        std::string gravity;
        std::string state;
        std::string fault;
        ui::theme::StatusLevel level{ ui::theme::StatusLevel::Neutral };
        float limitFraction{ 0.5f };
    };

    std::string_view StateName(can::FocMotorState state);
    std::string_view FaultName(can::FocFaultCode fault);
    ui::theme::StatusLevel LevelOf(const arm::JointReading& reading);
    bool ModelIsLive(const arm_host::ArmSnapshot& snapshot);
    std::array<JointRow, arm::jointCount> JointRows(const arm_host::ArmSnapshot& snapshot, const arm::JointLimits& limits, const arm::JointFlags& torqueLimited = {});
}
