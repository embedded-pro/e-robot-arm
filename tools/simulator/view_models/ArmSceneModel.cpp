#include "tools/simulator/view_models/ArmSceneModel.hpp"
#include "tools/simulator/view_models/JointRows.hpp"
#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

namespace simulator
{
    namespace
    {
        float Degrees(foc::Radians angle)
        {
            return angle.Value() * 180.0f / std::numbers::pi_v<float>;
        }

        std::array<float, 3> Rotate(const std::array<float, 4>& q, const std::array<float, 3>& v)
        {
            const std::array<float, 3> u{ q[1], q[2], q[3] };
            const std::array<float, 3> t{ 2.0f * (u[1] * v[2] - u[2] * v[1]), 2.0f * (u[2] * v[0] - u[0] * v[2]), 2.0f * (u[0] * v[1] - u[1] * v[0]) };
            return { v[0] + q[0] * t[0] + (u[1] * t[2] - u[2] * t[1]),
                v[1] + q[0] * t[1] + (u[2] * t[0] - u[0] * t[2]),
                v[2] + q[0] * t[2] + (u[0] * t[1] - u[1] * t[0]) };
        }

        std::array<float, 3> ToScenePoint(const std::array<float, 3>& modelPoint)
        {
            return { 100.0f * modelPoint[0], 100.0f * modelPoint[2], -100.0f * modelPoint[1] };
        }

        void Include(ArmSceneModel::Bounds& bounds, const std::array<float, 3>& point)
        {
            for (std::size_t axis = 0; axis != 3; ++axis)
            {
                bounds.min[axis] = std::min(bounds.min[axis], point[axis]);
                bounds.max[axis] = std::max(bounds.max[axis], point[axis]);
            }
        }

        void IncludeMesh(ArmSceneModel::Bounds& bounds, const ui::stage::ShadedMesh& mesh, const ScenePose& pose)
        {
            const std::array<std::array<float, 3>, 2> extremes{ { { mesh.boundsMin.x, mesh.boundsMin.y, mesh.boundsMin.z },
                { mesh.boundsMax.x, mesh.boundsMax.y, mesh.boundsMax.z } } };
            for (std::size_t corner = 0; corner != 8; ++corner)
            {
                const std::array<float, 3> local{ 100.0f * extremes[corner & 1u][0], 100.0f * extremes[(corner >> 1u) & 1u][1], 100.0f * extremes[(corner >> 2u) & 1u][2] };
                const auto rotated = Rotate(pose.rotation, local);
                Include(bounds, { pose.position[0] + rotated[0], pose.position[1] + rotated[1], pose.position[2] + rotated[2] });
            }
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
        UpdateBoxes(snapshot);
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
        UpdateTarget(model, ModelIsLive(snapshot));
        UpdateLimitArc(model, snapshot, measuredJoints);
        UpdateSelection(model, snapshot, measuredJoints);
        UpdateBounds();
    }

    void ArmSceneModel::UpdateSelection(const arm::ArmModel& model, const arm_host::ArmSnapshot& snapshot, const arm::JointPositions& measuredJoints)
    {
        std::vector<std::array<float, 3>> ring;
        selection.reset();
        if (selected.has_value())
        {
            const auto joint = *selected;
            const auto radius = selectionRingShare * extent / 100.0f;
            auto joints = measuredJoints;
            for (std::size_t point = 0; point != selectionRingPoints; ++point)
            {
                const auto share = static_cast<float>(point) / static_cast<float>(selectionRingPoints - 1);
                joints[joint] = foc::Radians{ measuredJoints[joint].Value() + 2.0f * std::numbers::pi_v<float> * share };
                const auto frame = model.LinkPoses(joints)[joint];
                const auto& t = frame.translation;
                const auto& r = frame.rotation;
                ring.push_back(ToScenePoint({ t[0] + radius * r.at(0, 0), t[1] + radius * r.at(1, 0), t[2] + radius * r.at(2, 0) }));
            }

            auto text = std::format("J{} \u00b7 measured {:.2f} rad", joint + 1, measuredJoints[joint].Value());
            if (ghostVisible && snapshot.commanded.has_value())
                text += std::format(" \u00b7 commanded {:.2f} rad", snapshot.commanded->at(joint).Value());
            selection = SelectionLabel{ std::move(text), measuredNodes[joint + 1].position };
        }
        if (ring != selectionRing)
        {
            selectionRing = std::move(ring);
            ++selectionRingRevision;
        }
    }

    void ArmSceneModel::UpdateBounds()
    {
        Bounds bounds{ toolNode.position, toolNode.position };
        const auto reach = toolFrameShare * extent;
        for (std::size_t axis = 0; axis != 3; ++axis)
        {
            bounds.min[axis] -= reach;
            bounds.max[axis] += reach;
        }
        for (std::size_t node = 0; node != armNodeCount; ++node)
            for (const auto& mesh : meshes[node])
                if (!mesh.vertices.empty())
                    IncludeMesh(bounds, mesh, measuredNodes[node]);
        frameBounds = bounds;
    }

    void ArmSceneModel::SetTarget(const std::optional<arm::JointPositions>& joints)
    {
        target = joints;
    }

    void ArmSceneModel::UpdateTarget(const arm::ArmModel& model, bool live)
    {
        targetNode.reset();
        if (!live || !target.has_value())
            return;

        const auto pose = ToScene(model.ToolPose(*target));
        float squared{ 0.0f };
        for (std::size_t axis = 0; axis != 3; ++axis)
            squared += (pose.position[axis] - toolNode.position[axis]) * (pose.position[axis] - toolNode.position[axis]);
        if (squared > trailSpacingCentimetres * trailSpacingCentimetres)
            targetNode = pose;
    }

    void ArmSceneModel::UpdateLimitArc(const arm::ArmModel& model, const arm_host::ArmSnapshot& snapshot, const arm::JointPositions& measuredJoints)
    {
        std::vector<std::array<float, 3>> arc;
        if (selected.has_value())
        {
            const auto joint = *selected;
            const auto& limit = snapshot.config.positionLimits[joint];
            const auto radius = limitArcShare * extent / 100.0f;
            auto joints = measuredJoints;
            for (std::size_t point = 0; point != limitArcPoints; ++point)
            {
                const auto share = static_cast<float>(point) / static_cast<float>(limitArcPoints - 1);
                joints[joint] = foc::Radians{ limit.lower.Value() + share * (limit.upper.Value() - limit.lower.Value()) };
                const auto frame = model.LinkPoses(joints)[joint];
                const auto& t = frame.translation;
                const auto& r = frame.rotation;
                arc.push_back(ToScenePoint({ t[0] + radius * r.at(0, 0), t[1] + radius * r.at(1, 0), t[2] + radius * r.at(2, 0) }));
            }
        }
        if (arc != limitArc)
        {
            limitArc = std::move(arc);
            ++limitArcRevision;
        }
    }

    void ArmSceneModel::UpdateBoxes(const arm_host::ArmSnapshot& snapshot)
    {
        const auto& zones = snapshot.config.workspace;
        BoxList built;
        built.reserve(1 + zones.forbiddenZoneCount);
        built.push_back(ToSceneBox(snapshot.workspace, SceneBoxKind::workspace));
        for (std::size_t zone = 0; zone != std::min<std::size_t>(zones.forbiddenZoneCount, arm::maxForbiddenZones); ++zone)
            built.push_back(ToSceneBox(zones.forbiddenZones[zone], SceneBoxKind::zone));

        if (snapshot.stop.has_value() && snapshot.stop->workspace.has_value())
        {
            const auto& violation = *snapshot.stop->workspace;
            if (violation.rule == arm::WorkspaceRule::workspace)
                built.front().violated = true;
            else if (violation.rule == arm::WorkspaceRule::forbiddenZone && violation.other + 1u < built.size())
                built[violation.other + 1u].violated = true;
        }

        if (built != boxes)
        {
            boxes = std::move(built);
            ++boxesRevision;
        }
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

    const ArmSceneModel::BoxList& ArmSceneModel::Boxes() const
    {
        return boxes;
    }

    const std::optional<ScenePose>& ArmSceneModel::TargetNode() const
    {
        return targetNode;
    }

    const ArmSceneModel::Bounds& ArmSceneModel::FrameBounds() const
    {
        return frameBounds;
    }

    const std::vector<std::array<float, 3>>& ArmSceneModel::SelectionRing() const
    {
        return selectionRing;
    }

    uint32_t ArmSceneModel::SelectionRingRevision() const
    {
        return selectionRingRevision;
    }

    const std::optional<ArmSceneModel::SelectionLabel>& ArmSceneModel::Selection() const
    {
        return selection;
    }

    const std::vector<std::array<float, 3>>& ArmSceneModel::LimitArc() const
    {
        return limitArc;
    }

    uint32_t ArmSceneModel::LimitArcRevision() const
    {
        return limitArcRevision;
    }

    uint32_t ArmSceneModel::BoxesRevision() const
    {
        return boxesRevision;
    }
}
