#ifndef TOOLS_SIMULATOR_VIEW_MODELS_TOOL_READOUT_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_TOOL_READOUT_HPP

#include "tools/arm_host/ArmSnapshot.hpp"
#include "ui/theme/Theme.hpp"
#include <string>

namespace simulator
{
    struct ToolReadout
    {
        std::string position;
        std::string rotation;
        std::string manipulability;
        std::string singularity;
        ui::theme::StatusLevel singularityLevel{ ui::theme::StatusLevel::Neutral };
        std::string warning;

        bool operator==(const ToolReadout& other) const = default;
    };

    ToolReadout ToolReadoutOf(const arm_host::ArmSnapshot& snapshot);
}

#endif
