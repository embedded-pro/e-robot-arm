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

        bool SameGeometry(const arm::ArmGeometry& a, const arm::ArmGeometry& b)
        {
            const auto& p = a.opw;
            const auto& q = b.opw;
            return p.a1 == q.a1 && p.a2 == q.a2 && p.b == q.b && p.c1 == q.c1 && p.c2 == q.c2 && p.c3 == q.c3 && p.c4 == q.c4;
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
        singularityWarning = ToolReadoutOf(snapshot).warning;
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
        toolNode = ToScene(model.ToolPose(measuredJoints));

        const bool rebuilt = !builtFor || !SameGeometry(*builtFor, geometry);
        if (rebuilt)
        {
            meshes = BuildDesktopArm(geometry);
            builtFor = geometry;
            extent = ArmExtent(geometry);
            ++meshesRevision;
        }
        UpdateTrail(ModelIsLive(snapshot), rebuilt);
    }

    void ArmSceneModel::UpdateTrail(bool live, bool rebuilt)
    {
        if (!live || rebuilt)
        {
            if (!trail.empty())
            {
                trail.clear();
                ++trailRevision;
            }
            if (!live)
                return;
        }

        const auto& point = toolNode.position;
        if (!trail.empty())
        {
            const auto& last = trail.back();
            const auto dx = point[0] - last[0];
            const auto dy = point[1] - last[1];
            const auto dz = point[2] - last[2];
            if (dx * dx + dy * dy + dz * dz < trailSpacingCentimetres * trailSpacingCentimetres)
                return;
        }

        trail.push_back(point);
        if (trail.size() > trailCapacity)
            trail.pop_front();
        ++trailRevision;
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

    const ArmMeshes& ArmSceneModel::Meshes() const
    {
        return meshes;
    }

    uint32_t ArmSceneModel::MeshesRevision() const
    {
        return meshesRevision;
    }

    float ArmSceneModel::Extent() const
    {
        return extent;
    }

    const std::string& ArmSceneModel::SingularityWarning() const
    {
        return singularityWarning;
    }

    const ScenePose& ArmSceneModel::ToolNode() const
    {
        return toolNode;
    }

    const ArmSceneModel::Trail& ArmSceneModel::ToolTrail() const
    {
        return trail;
    }

    uint32_t ArmSceneModel::TrailRevision() const
    {
        return trailRevision;
    }
}
