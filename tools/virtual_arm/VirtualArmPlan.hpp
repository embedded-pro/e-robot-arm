#pragma once

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

    struct JointPlant
    {
        std::string name;
        std::string description;

        bool operator==(const JointPlant& other) const = default;
    };

    struct MotorStart
    {
        std::size_t joint{ 0 };
        uint8_t boardId{ 0 };
        JointPlant plant;
        std::string plantFile;
        std::string memoryFile;
        uint32_t seed{ 0 };
        bool hostsBus{ false };
    };

    using VirtualArmPlan = std::array<MotorStart, arm::jointCount>;
    using LoadInertias = std::array<float, arm::jointCount>;

    LoadInertias LinkInertias(const arm::ArmConfig& config);
    JointPlant PlantOf(const std::optional<arm::JointMotor>& motor, float loadInertia);
    std::variant<VirtualArmPlan, std::string> PlanVirtualArm(const arm::ArmConfig& config);
    std::vector<std::string> EngineArguments(const MotorStart& motor, std::string_view bus, uint16_t plantPort, uint16_t hubPort);
}
