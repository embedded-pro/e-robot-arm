#include "tools/virtual_arm/VirtualArmPlan.hpp"
#include "core/arm_model/ArmModel.hpp"
#include <format>

namespace virtual_arm
{
    namespace
    {
        struct DescribedValue
        {
            arm::MotorParameter parameter;
            std::string_view key;
            float toSi;
        };

        constexpr std::array<DescribedValue, 8> describedValues{ {
            { arm::MotorParameter::resistance, "resistance", 1.0f },
            { arm::MotorParameter::inductance, "inductance", 1.0e-3f },
            { arm::MotorParameter::fluxLinkage, "flux-linkage", 1.0f },
            { arm::MotorParameter::polePairs, "pole-pairs", 1.0f },
            { arm::MotorParameter::inertia, "rotor-inertia", 1.0f },
            { arm::MotorParameter::viscousFriction, "viscous-friction", 1.0f },
            { arm::MotorParameter::coulombFriction, "coulomb-friction", 1.0f },
            { arm::MotorParameter::peakCurrent, "max-current", 1.0f },
        } };

        std::string_view ShortName(const arm::MotorPreset& preset)
        {
            return preset.key.substr(0, preset.key.find('-'));
        }

        bool KnownPreset(const std::optional<arm::JointMotor>& motor)
        {
            return motor.has_value() && motor->preset.has_value() && *motor->preset < arm::motorPresets.size();
        }
    }

    LoadInertias LinkInertias(const arm::ArmConfig& config)
    {
        const arm::ArmModel model{ config.geometry, config.mass, config.positionLimits };
        const auto mass = model.JointMassMatrix(arm::JointPositions{});
        LoadInertias inertias{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto gain = config.transmissions[joint].Gain();
            inertias[joint] = mass.at(joint, joint) / (gain * gain);
        }
        return inertias;
    }

    JointPlant PlantOf(const std::optional<arm::JointMotor>& motor, float loadInertia)
    {
        const auto& preset = arm::motorPresets[KnownPreset(motor) ? *motor->preset : 0];
        const auto presetName = ShortName(preset);
        JointPlant plant{ std::string{ !motor.has_value() || KnownPreset(motor) ? presetName : "custom" },
            std::format("e-foc plant description 1\npreset {}\n", presetName) };

        if (motor.has_value())
            for (const auto& value : describedValues)
                if (!arm::MotorValuesMatch(preset.settings[value.parameter], motor->settings[value.parameter]))
                    plant.description += std::format("{} {}\n", value.key, motor->settings[value.parameter] * value.toSi);

        plant.description += std::format("load-inertia {}\n", loadInertia);
        return plant;
    }

    std::variant<VirtualArmPlan, std::string> PlanVirtualArm(const arm::ArmConfig& config)
    {
        const auto inertias = LinkInertias(config);
        VirtualArmPlan plan{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto node = config.nodes[joint];
            if (node < 1 || node > 256)
                return std::format("joint {} has node {}: a virtual motor answers on nodes 1 to 256", joint + 1, node);

            plan[joint] = { joint, static_cast<uint8_t>(node - 1), PlantOf(config.motors[joint], inertias[joint]), std::format("joint{}.plant", joint + 1),
                std::format("joint{}.mem", joint + 1), static_cast<uint32_t>(joint + 1), joint == 0 };
        }
        return plan;
    }

    std::vector<std::string> EngineArguments(const MotorStart& motor, std::string_view bus, uint16_t plantPort, uint16_t hubPort)
    {
        std::vector<std::string> arguments{ "--bus", std::string{ bus }, "--node", std::to_string(motor.boardId), "--plant", motor.plantFile, "--memory",
            motor.memoryFile, "--calibrated", "--seed", std::to_string(motor.seed), "--plant-port", std::to_string(plantPort), "--hub-port", std::to_string(hubPort) };
        if (motor.hostsBus)
            arguments.emplace_back("--host-bus");
        return arguments;
    }
}
