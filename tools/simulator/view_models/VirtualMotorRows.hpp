#pragma once

#include "tools/arm_host/ArmSnapshot.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <string>

namespace simulator
{
    struct VirtualMotorRow
    {
        std::string joint;
        std::string motor;
        std::string process;
        ui::theme::StatusLevel level{ ui::theme::StatusLevel::Neutral };
        bool restartEnabled{ false };

        friend bool operator==(const VirtualMotorRow& lhs, const VirtualMotorRow& rhs) = default;
    };

    struct VirtualMotorsView
    {
        bool visible{ false };
        std::array<VirtualMotorRow, arm::jointCount> rows{};
        std::string folder;
        std::string hint;

        friend bool operator==(const VirtualMotorsView& lhs, const VirtualMotorsView& rhs) = default;
    };

    VirtualMotorsView ViewVirtualMotors(const arm_host::ArmSnapshot& snapshot);
    std::string StartingVirtualMotors(const arm_host::ArmSnapshot& snapshot);
}
