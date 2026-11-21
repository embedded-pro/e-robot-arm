#pragma once

#include "core/arm_model/ArmModel.hpp"
#include "core/workspace/WorkspaceConfig.hpp"
#include <array>
#include <cstdint>

namespace simulator
{
    struct ScenePose
    {
        std::array<float, 3> position{};
        std::array<float, 4> rotation{ 1.0f, 0.0f, 0.0f, 0.0f };

        bool operator==(const ScenePose& other) const = default;
    };

    enum class SceneBoxKind : uint8_t
    {
        workspace,
        zone
    };

    struct SceneBox
    {
        SceneBoxKind kind{ SceneBoxKind::workspace };
        std::array<float, 3> centre{};
        std::array<float, 3> size{};
        bool violated{ false };

        bool operator==(const SceneBox& other) const = default;
    };

    SceneBox ToSceneBox(const arm::WorkspaceBox& box, SceneBoxKind kind);
    std::array<std::array<float, 3>, 16> BoxOutline(const SceneBox& box);

    constexpr std::size_t sceneNodeCount{ arm::jointCount + 1 };
    using SceneNodes = std::array<ScenePose, sceneNodeCount>;

    ScenePose ToScene(const arm::ArmModel::Pose& pose);
    SceneNodes SceneNodePoses(const arm::ArmModel& model, const arm::ArmGeometry& geometry, const arm::JointPositions& joints);
    float ArmExtent(const arm::ArmGeometry& geometry);
}
