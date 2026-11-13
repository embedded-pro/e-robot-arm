#include "tools/simulator/view_models/ArmSceneModel.hpp"
#include "tools/simulator/view_models/JointRows.hpp"
#include <cmath>
#include <numbers>

namespace simulator
{
    namespace
    {
        float Degrees(foc::Radians angle)
        {
            return angle.Value() * 180.0f / std::numbers::pi_v<float>;
        }
    }

    void ArmSceneModel::Update(const arm_host::ArmSnapshot& snapshot)
    {
        bool apart = false;
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            measured[joint] = Degrees(snapshot.readings[joint].position);
            commanded[joint] = snapshot.commanded ? Degrees(snapshot.commanded->at(joint)) : measured[joint];
            apart = apart || std::abs(commanded[joint] - measured[joint]) > ghostThresholdDegrees;
            levels[joint] = snapshot.targetReady ? LevelOf(snapshot.readings[joint]) : ui::theme::StatusLevel::Neutral;
        }
        ghostVisible = snapshot.targetReady && (snapshot.moving || apart);
        UpdateNodes(snapshot);
    }

    void ArmSceneModel::UpdateNodes(const arm_host::ArmSnapshot& snapshot)
    {
        const auto& geometry = snapshot.config.geometry;
        const arm::ArmModel model{ geometry, snapshot.config.mass, snapshot.config.positionLimits };
        arm::JointPositions measuredJoints{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            measuredJoints[joint] = snapshot.readings[joint].position;

        measuredNodes = SceneNodePoses(model, geometry, measuredJoints);
        commandedNodes = SceneNodePoses(model, geometry, snapshot.commanded.value_or(measuredJoints));

        auto built = BuildSceneParts(geometry);
        if (built != parts)
        {
            parts = std::move(built);
            extent = ArmExtent(geometry);
            ++partsRevision;
        }
    }

    bool ArmSceneModel::Select(int pickedJoint)
    {
        if (pickedJoint < 0 || pickedJoint >= static_cast<int>(arm::jointCount))
            return false;

        selected = static_cast<std::size_t>(pickedJoint);
        return true;
    }

    void ArmSceneModel::ClearSelection()
    {
        selected.reset();
    }

    const std::array<float, arm::jointCount>& ArmSceneModel::MeasuredDegrees() const
    {
        return measured;
    }

    const std::array<float, arm::jointCount>& ArmSceneModel::CommandedDegrees() const
    {
        return commanded;
    }

    bool ArmSceneModel::GhostVisible() const
    {
        return ghostVisible;
    }

    std::optional<std::size_t> ArmSceneModel::Selected() const
    {
        return selected;
    }

    ui::theme::StatusLevel ArmSceneModel::Level(std::size_t joint) const
    {
        return levels[joint];
    }

    const SceneNodes& ArmSceneModel::MeasuredNodes() const
    {
        return measuredNodes;
    }

    const SceneNodes& ArmSceneModel::CommandedNodes() const
    {
        return commandedNodes;
    }

    const SceneParts& ArmSceneModel::Parts() const
    {
        return parts;
    }

    uint32_t ArmSceneModel::PartsRevision() const
    {
        return partsRevision;
    }

    float ArmSceneModel::Extent() const
    {
        return extent;
    }
}
