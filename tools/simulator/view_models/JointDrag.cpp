#include "tools/simulator/view_models/JointDrag.hpp"
#include <algorithm>
#include <cmath>
#include <format>

namespace simulator
{
    namespace
    {
        constexpr float angleStep{ 1.0e-3f };
        constexpr float smallestArc{ 1.0e-10f };

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
        : post(post)
    {}

    bool JointDrag::FollowOffered(const arm_host::ArmSnapshot& snapshot, const ControlState& move)
    {
        return move.enabled && snapshot.disturbances.has_value();
    }

    bool JointDrag::Grab(int pickedJoint, const ScenePoint& hit, const arm_host::ArmSnapshot& snapshot, const ControlState& move, bool follow)
    {
        if (pickedJoint < 0 || pickedJoint >= static_cast<int>(arm::jointCount) || !snapshot.targetReady)
            return false;

        StopFollowing();
        model.emplace(snapshot.config.geometry, snapshot.config.mass, snapshot.config.positionLimits);
        joint = static_cast<std::size_t>(pickedJoint);
        start = Measured(snapshot);
        local = model->LinkPoses(start)[*joint].Inverse().Apply(ToBase(hit));
        lower = snapshot.config.positionLimits[*joint].lower;
        upper = snapshot.config.positionLimits[*joint].upper;
        following = follow && FollowOffered(snapshot, move);
        followRefused = follow && !following;
        atLimit = false;
        dragging = true;
        target.reset();
        lastFollow.reset();
        return true;
    }

    void JointDrag::Drag(const SceneRay& ray, const arm_host::ArmSnapshot& snapshot, const ControlState& move, Clock::time_point now)
    {
        if (!dragging)
            return;
        if (following && !FollowOffered(snapshot, move))
        {
            Release();
            return;
        }

        const auto from = target.has_value() ? (*target)[*joint].Value() : start[*joint].Value();
        const auto angle = Solve(ray, from);
        target = start;
        (*target)[*joint] = foc::Radians{ angle };

        if (following && (!lastFollow.has_value() || now - *lastFollow >= followPeriod))
        {
            lastFollow = now;
            post(arm_host::MoveTo{ *target });
        }
    }

    void JointDrag::Release()
    {
        if (!dragging)
            return;
        dragging = false;
        if (following)
        {
            StopFollowing();
            target.reset();
        }
    }

    void JointDrag::Cancel()
    {
        Release();
        target.reset();
        joint.reset();
    }

    bool JointDrag::Confirm(const ControlState& move)
    {
        if (dragging || !target.has_value() || !move.enabled)
            return false;
        post(arm_host::MoveTo{ *target });
        target.reset();
        joint.reset();
        return true;
    }

    bool JointDrag::Dragging() const
    {
        return dragging;
    }

    bool JointDrag::Following() const
    {
        return following;
    }

    const std::optional<arm::JointPositions>& JointDrag::Target() const
    {
        return target;
    }

    std::string JointDrag::Label(const ControlState& move) const
    {
        if (!target.has_value())
            return {};
        auto label = std::format("J{} → {:.2f} rad (from {:.2f})", *joint + 1, (*target)[*joint].Value(), start[*joint].Value());
        if (atLimit)
            label += " · at limit";
        if (following)
            return label + " · following";
        if (followRefused)
            label += " · follow: simulation only";
        if (move.enabled)
            return label + " · Enter: move · Esc: cancel";
        return label + " · Move: " + move.reason;
    }

    math::Vector3<float> JointDrag::PointAt(float angle) const
    {
        auto joints = start;
        joints[*joint] = foc::Radians{ angle };
        return model->LinkPoses(joints)[*joint].Apply(local);
    }

    float JointDrag::Solve(const SceneRay& ray, float angle)
    {
        const auto origin = ToBase(ray.origin);
        auto direction = ToBase(ray.direction);
        const auto length = std::sqrt(Dot(direction, direction));
        if (length > 0.0f)
            direction = direction * (1.0f / length);
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
            const auto next = angle - Dot(tangent, miss) / arc;
            angle = std::clamp(next, lower.Value(), upper.Value());
            atLimit = angle != next;
        }
        return angle;
    }

    void JointDrag::StopFollowing()
    {
        if (following)
            post(arm_host::AbortMove{});
        following = false;
    }
}
