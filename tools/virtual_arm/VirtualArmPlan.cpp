#include "tools/virtual_arm/VirtualArmPlan.hpp"
#include <format>

namespace virtual_arm
{
    std::optional<std::string> PlantOf(const std::optional<arm::JointMotor>& motor)
    {
        if (!motor.has_value())
            return std::string{ defaultPlant };
        if (!motor->preset.has_value() || *motor->preset >= arm::motorPresets.size())
            return std::nullopt;

        const auto key = arm::motorPresets[*motor->preset].key;
        return std::string{ key.substr(0, key.find('-')) };
    }

    std::variant<VirtualArmPlan, std::string> PlanVirtualArm(const arm::ArmConfig& config)
    {
        VirtualArmPlan plan{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto node = config.nodes[joint];
            if (node < 1 || node > 256)
                return std::format("joint {} has node {}: a virtual motor answers on nodes 1 to 256", joint + 1, node);

            const auto plant = PlantOf(config.motors[joint]);
            if (!plant.has_value())
                return std::format("joint {} has a custom motor: a virtual motor takes e-foc's plant presets only", joint + 1);

            plan[joint] = { joint, static_cast<uint8_t>(node - 1), *plant, std::format("joint{}.mem", joint + 1), static_cast<uint32_t>(joint + 1), joint == 0 };
        }
        return plan;
    }

    std::vector<std::string> EngineArguments(const MotorStart& motor, std::string_view bus, uint16_t plantPort)
    {
        std::vector<std::string> arguments{ "--bus", std::string{ bus }, "--node", std::to_string(motor.boardId), "--plant", motor.plant, "--memory",
            motor.memoryFile, "--calibrated", "--seed", std::to_string(motor.seed), "--plant-port", std::to_string(plantPort) };
        if (motor.hostsBus)
            arguments.emplace_back("--host-bus");
        return arguments;
    }
}
