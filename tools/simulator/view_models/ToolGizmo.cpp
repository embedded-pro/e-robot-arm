#include "tools/simulator/view_models/ToolGizmo.hpp"
#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

namespace simulator
{
    namespace
    {
        constexpr float parallel{ 1.0e-6f };
        constexpr float metresToCentimetres{ 100.0f };

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

        // Where along the line through point with unit direction axis the ray passes closest.
        std::optional<float> AlongLine(const math::Vector3<float>& point, const math::Vector3<float>& axis, const BaseRay& ray)
        {
            const auto offset = point - ray.origin;
            const auto cosine = Dot(axis, ray.direction);
            const auto denominator = 1.0f - cosine * cosine;
            if (denominator < parallel)
                return std::nullopt;
            return (cosine * Dot(ray.direction, offset) - Dot(axis, offset)) / denominator;
        }

        std::optional<math::Vector3<float>> OnPlane(const math::Vector3<float>& point, const math::Vector3<float>& normal, const BaseRay& ray)
        {
            const auto facing = Dot(normal, ray.direction);
            if (std::abs(facing) < parallel)
                return std::nullopt;
            return ray.origin + ray.direction * (Dot(normal, point - ray.origin) / facing);
        }

        float RotationAngle(const math::Matrix3<float>& rotation)
        {
            const auto cosine = (rotation.at(0, 0) + rotation.at(1, 1) + rotation.at(2, 2) - 1.0f) / 2.0f;
            return std::acos(std::clamp(cosine, -1.0f, 1.0f));
        }

        std::string NearNames(const arm::Singularities& close)
        {
            std::string names;
            for (const auto& [is, name] : { std::pair{ close.wrist, "wrist" }, std::pair{ close.elbow, "elbow" }, std::pair{ close.shoulder, "shoulder" } })
                if (is)
                    names += (names.empty() ? "" : ", ") + std::string{ name };
            return names;
        }

        bool Turning(GizmoHandle handle)
        {
            return handle == GizmoHandle::turnX || handle == GizmoHandle::turnY || handle == GizmoHandle::turnZ;
        }

        math::Vector3<float> Column(const math::Matrix3<float>& rotation, std::size_t index)
        {
            return math::Vector3<float>{ rotation.at(0, index), rotation.at(1, index), rotation.at(2, index) };
        }

        math::Vector3<float> AxisOf(GizmoHandle handle, const arm::ArmModel::Pose& start)
        {
            const auto index = static_cast<std::size_t>(handle) % 3;
            if (Turning(handle))
                return Column(start.rotation, index);
            math::Vector3<float> axis{};
            axis[index] = 1.0f;
            return axis;
        }

        // Moving: how far along the arrow the pointer is. Turning: the pointer's angle about the axis.
        std::optional<float> ReadingOf(GizmoHandle handle, const arm::ArmModel::Pose& start, const BaseRay& ray)
        {
            const auto axis = AxisOf(handle, start);
            if (!Turning(handle))
                return AlongLine(start.translation, axis, ray);

            const auto hit = OnPlane(start.translation, axis, ray);
            if (!hit.has_value())
                return std::nullopt;
            const auto offset = *hit - start.translation;
            const auto reference = Column(start.rotation, (static_cast<std::size_t>(handle) + 1) % 3);
            return std::atan2(Dot(offset, math::CrossProduct(axis, reference)), Dot(offset, reference));
        }
    }

    ToolGizmo::ToolGizmo(const ui::Callback<void(const arm_host::ArmCommand& command)>& post)
        : pointer(post)
    {}

    bool ToolGizmo::Grab(GizmoHandle grabbedHandle, const SceneRay& ray, const arm_host::ArmSnapshot& snapshot, const ControlState& move, bool follow)
    {
        if (!snapshot.targetReady)
            return false;
        arm::ArmModel grabbedModel{ snapshot.config.geometry, snapshot.config.mass, snapshot.config.positionLimits };
        const auto measured = Measured(snapshot);
        const auto pose = grabbedModel.ToolPose(measured);
        const auto reading = ReadingOf(grabbedHandle, pose, ToBaseRay(ray));
        if (!reading.has_value())
            return false;

        model.emplace(grabbedModel);
        geometry = snapshot.config.geometry;
        handle = grabbedHandle;
        solution = measured;
        start = pose;
        wanted = pose;
        failure.reset();
        startReading = *reading;
        pointer.Begin(follow, snapshot, move);
        return true;
    }

    void ToolGizmo::Drag(const SceneRay& ray, const arm_host::ArmSnapshot& snapshot, const ControlState& move, Clock::time_point now)
    {
        if (!pointer.Dragging() || !pointer.KeepFollowing(snapshot, move))
            return;
        const auto reading = ReadingOf(*handle, start, ToBaseRay(ray));
        if (!reading.has_value())
            return;

        wanted = Wanted(*reading);
        const auto result = model->Inverse(wanted, solution);
        if (result.joints.has_value())
        {
            failure.reset();
            solution = *result.joints;
        }
        else
            failure = result.failure;
        pointer.Set(solution, now, Reachable());
    }

    void ToolGizmo::Release()
    {
        pointer.Release();
    }

    void ToolGizmo::Cancel()
    {
        pointer.Cancel();
    }

    bool ToolGizmo::Confirm(const ControlState& move)
    {
        return pointer.Confirm(move);
    }

    bool ToolGizmo::Dragging() const
    {
        return pointer.Dragging();
    }

    bool ToolGizmo::Following() const
    {
        return pointer.Following();
    }

    bool ToolGizmo::Reachable() const
    {
        return !failure.has_value();
    }

    std::optional<GizmoHandle> ToolGizmo::Handle() const
    {
        return pointer.Target().has_value() || pointer.Dragging() ? handle : std::nullopt;
    }

    const std::optional<arm::JointPositions>& ToolGizmo::Target() const
    {
        return pointer.Target();
    }

    std::optional<SceneNodes> ToolGizmo::TargetNodes() const
    {
        if (!pointer.Target().has_value())
            return std::nullopt;
        return SceneNodePoses(*model, geometry, *pointer.Target());
    }

    std::optional<ScenePoint> ToolGizmo::TargetPoint() const
    {
        if (!pointer.Target().has_value())
            return std::nullopt;
        return ToScenePoint(wanted.translation);
    }

    std::string ToolGizmo::Label(const ControlState& move) const
    {
        if (!pointer.Target().has_value())
            return {};
        const auto& to = wanted.translation;
        const auto shift = to - start.translation;
        const auto turn = RotationAngle(wanted.rotation * start.rotation.Transpose()) * 180.0f / std::numbers::pi_v<float>;
        auto label = std::format("Tool → x {:.3f}  y {:.3f}  z {:.3f} m · Δ {:.1f} cm {:.0f}°", to[0], to[1], to[2],
            std::sqrt(Dot(shift, shift)) * metresToCentimetres, turn);
        if (failure == arm::IkFailure::unreachable)
            label += " · out of reach";
        else if (failure == arm::IkFailure::outsideLimits)
            label += " · outside the joint limits";
        else if (const auto close = model->Near(solution); close.Any())
            label += " · near " + NearNames(close);
        return label + pointer.Suffix(move);
    }

    ui::theme::StatusLevel ToolGizmo::Level() const
    {
        if (!pointer.Target().has_value())
            return ui::theme::StatusLevel::Neutral;
        if (!Reachable())
            return ui::theme::StatusLevel::Fault;
        return model->Near(solution).Any() ? ui::theme::StatusLevel::Warning : ui::theme::StatusLevel::Neutral;
    }

    arm::ArmModel::Pose ToolGizmo::Wanted(float reading) const
    {
        auto pose = start;
        const auto axis = AxisOf(*handle, start);
        if (Turning(*handle))
            pose.rotation = math::RotationAboutAxis(axis, reading - startReading) * start.rotation;
        else
            pose.translation = start.translation + axis * (reading - startReading);
        return pose;
    }
}
