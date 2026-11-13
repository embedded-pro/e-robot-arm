#ifndef TOOLS_SIMULATOR_VIEW_MODELS_ARM_SCENE_GEOMETRY_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_ARM_SCENE_GEOMETRY_HPP

#include "core/arm_model/ArmModel.hpp"
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

    constexpr std::size_t sceneNodeCount{ arm::jointCount + 1 };
    using SceneNodes = std::array<ScenePose, sceneNodeCount>;

    ScenePose ToScene(const arm::ArmModel::Pose& pose);
    SceneNodes SceneNodePoses(const arm::ArmModel& model, const arm::ArmGeometry& geometry, const arm::JointPositions& joints);
    float ArmExtent(const arm::ArmGeometry& geometry);
}

#endif
