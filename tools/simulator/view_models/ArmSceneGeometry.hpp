#ifndef TOOLS_SIMULATOR_VIEW_MODELS_ARM_SCENE_GEOMETRY_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_ARM_SCENE_GEOMETRY_HPP

#include "core/arm_model/ArmModel.hpp"
#include <array>
#include <cstdint>
#include <vector>

namespace simulator
{
    enum class PartShape : uint8_t
    {
        cylinder,
        box
    };

    enum class PartMaterial : uint8_t
    {
        cast,
        housing,
        paint,
        accent,
        rubber
    };

    struct ScenePose
    {
        std::array<float, 3> position{};
        std::array<float, 4> rotation{ 1.0f, 0.0f, 0.0f, 0.0f };

        bool operator==(const ScenePose& other) const = default;
    };

    struct ScenePart
    {
        int joint{ -1 };
        PartShape shape{ PartShape::box };
        PartMaterial material{ PartMaterial::paint };
        std::array<float, 3> centre{};
        std::array<float, 4> rotation{ 1.0f, 0.0f, 0.0f, 0.0f };
        std::array<float, 3> scale{};

        bool operator==(const ScenePart& other) const = default;
    };

    constexpr std::size_t sceneNodeCount{ arm::jointCount + 1 };
    using SceneNodes = std::array<ScenePose, sceneNodeCount>;
    using SceneParts = std::array<std::vector<ScenePart>, sceneNodeCount>;

    ScenePose ToScene(const arm::ArmModel::Pose& pose);
    SceneNodes SceneNodePoses(const arm::ArmModel& model, const arm::ArmGeometry& geometry, const arm::JointPositions& joints);
    SceneParts BuildSceneParts(const arm::ArmGeometry& geometry);
    float PartThickness(const arm::ArmGeometry& geometry);
    float ArmExtent(const arm::ArmGeometry& geometry);
}

#endif
