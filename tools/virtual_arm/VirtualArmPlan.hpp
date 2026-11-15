#ifndef TOOLS_VIRTUAL_ARM_VIRTUAL_ARM_PLAN_HPP
#define TOOLS_VIRTUAL_ARM_VIRTUAL_ARM_PLAN_HPP

#include "core/arm/ArmConfig.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace virtual_arm
{
    inline constexpr std::string_view defaultBus{ "arm" };
    inline constexpr std::string_view defaultPlant{ "teknic" };

    struct MotorStart
    {
        std::size_t joint{ 0 };
        uint8_t boardId{ 0 };
        std::string plant;
        std::string memoryFile;
        uint32_t seed{ 0 };
        bool hostsBus{ false };
    };

    using VirtualArmPlan = std::array<MotorStart, arm::jointCount>;

    std::optional<std::string> PlantOf(const std::optional<arm::JointMotor>& motor);
    std::variant<VirtualArmPlan, std::string> PlanVirtualArm(const arm::ArmConfig& config);
    std::vector<std::string> EngineArguments(const MotorStart& motor, std::string_view bus, uint16_t plantPort, uint16_t hubPort);
}

#endif
