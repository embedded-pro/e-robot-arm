#include "tools/simulator/view_models/JointDrag.hpp"
#include <algorithm>
#include <cmath>
#include <format>

namespace simulator
{
    namespace
    {
        constexpr float angleStep{ 1.0e-3f };
        // A point within 1 cm of the joint axis barely moves with the joint; its step would be noise.
        constexpr float smallestArc{ 1.0e-4f };
        constexpr float largestStep{ 0.5f };

        float Dot(const math::Vector3<float>& a, const math::Vector3<float>& b)
        {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        arm::JointPositions Measured(const arm_host::ArmSnapshot& snapshot)
        {
            arm::JointPositions joints{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                joints[joint] = snapshot.readings[joint].position;
            return joints;
        }
    }

    JointDrag::JointDrag(const ui::Callback<void(const arm_host::ArmCommand& command)>& post)
        : pointer(post)
    {}

    bool JointDrag::FollowOffered(const arm_host::ArmSnapshot& snapshot, const ControlState& move)
    {
        return PointerTarget::FollowOffered(snapshot, move);
    }

    bool JointDrag::Grab(int pickedJoint, const ScenePoint& hit, const arm_host::ArmSnapshot& snapshot, const ControlState& move, bool follow)
    {
        if (pickedJoint < 0 || pickedJoint >= static_cast<int>(arm::jointCount) || !snapshot.targetReady)
            return false;

        pointer.Begin(follow, snapshot, move);
        model.emplace(snapshot.config.geometry, snapshot.config.mass, snapshot.config.positionLimits);
        geometry = snapshot.config.geometry;
        joint = static_cast<std::size_t>(pickedJoint);
        start = Measured(snapshot);
        local = model->LinkPoses(start)[*joint].Inverse().Apply(ToBase(hit));
        lower = snapshot.config.positionLimits[*joint].lower;
        upper = snapshot.config.positionLimits[*joint].upper;
        atLimit = false;
        return true;
    }

    void JointDrag::Drag(const SceneRay& ray, const arm_host::ArmSnapshot& snapshot, const ControlState& move, Clock::time_point now)
    {
        if (!pointer.Dragging() || !pointer.KeepFollowing(snapshot, move))
            return;

        const auto& target = pointer.Target();
        const auto from = target.has_value() ? (*target)[*joint].Value() : start[*joint].Value();
        auto joints = start;
        joints[*joint] = foc::Radians{ Solve(ray, from) };
        pointer.Set(joints, now);
    }

    void JointDrag::Release()
    {
        pointer.Release();
    }

    void JointDrag::Cancel()
    {
        pointer.Cancel();
    }

    bool JointDrag::Confirm(const ControlState& move)
    {
        return pointer.Confirm(move);
    }

    bool JointDrag::Dragging() const
    {
        return pointer.Dragging();
    }

    bool JointDrag::Following() const
    {
        return pointer.Following();
    }

    const std::optional<arm::JointPositions>& JointDrag::Target() const
    {
        return pointer.Target();
    }

    std::string JointDrag::Label(const ControlState& move) const
    {
        const auto& target = pointer.Target();
        if (!target.has_value())
            return {};
        auto label = std::format("J{} → {:.2f} rad (from {:.2f})", *joint + 1, (*target)[*joint].Value(), start[*joint].Value());
        if (atLimit)
            label += " · at limit";
        return label + pointer.Suffix(move);
    }

    std::optional<SceneNodes> JointDrag::TargetNodes() const
    {
        if (!pointer.Target().has_value())
            return std::nullopt;
        return SceneNodePoses(*model, geometry, *pointer.Target());
    }

    std::optional<ScenePoint> JointDrag::TargetPoint() const
    {
        if (!pointer.Target().has_value())
            return std::nullopt;
        return ToScenePoint(PointAt((*pointer.Target())[*joint].Value()));
    }

    math::Vector3<float> JointDrag::PointAt(float angle) const
    {
        auto joints = start;
        joints[*joint] = foc::Radians{ angle };
        return model->LinkPoses(joints)[*joint].Apply(local);
    }

    float JointDrag::Solve(const SceneRay& ray, float angle)
    {
        const auto [origin, direction] = ToBaseRay(ray);
        const auto offRay = [&direction](const math::Vector3<float>& vector)
        {
            return vector - direction * Dot(vector, direction);
        };

        atLimit = false;
        for (std::size_t step = 0; step != solverSteps; ++step)
        {
            const auto point = PointAt(angle);
            const auto miss = offRay(point - origin);
            const auto tangent = offRay((PointAt(angle + angleStep) - point) * (1.0f / angleStep));
            const auto arc = Dot(tangent, tangent);
            if (arc < smallestArc)
                break;
            const auto next = angle + std::clamp(-Dot(tangent, miss) / arc, -largestStep, largestStep);
            angle = std::clamp(next, lower.Value(), upper.Value());
            atLimit = angle != next;
        }
        return angle;
    }
}
