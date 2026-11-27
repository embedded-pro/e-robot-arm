#pragma once

#include "core/arm_model/ArmModel.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ArmSceneGeometry.hpp"
#include "tools/simulator/view_models/ControlRules.hpp"
#include "tools/simulator/view_models/PointerTarget.hpp"
#include "tools/simulator/view_models/ScenePicking.hpp"
#include "ui/core/Callback.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace simulator
{
    enum class GizmoHandle : uint8_t
    {
        moveX,
        moveY,
        moveZ,
        turnX,
        turnY,
        turnZ
    };

    inline constexpr std::size_t gizmoHandleCount{ 6 };
    inline constexpr std::size_t gizmoRingPoints{ 48 };
    using GizmoLine = std::vector<ScenePoint>;

    // Each handle as a scene polyline: an arrow of length size, or a ring of 0.8 size about the tool axis.
    std::array<GizmoLine, gizmoHandleCount> GizmoLines(const arm::ArmModel::Pose& at, float size);
    // The handle the ray passes within 8 % of size of, nearest the ray's origin.
    std::optional<GizmoHandle> GizmoHandleAt(const SceneRay& ray, const arm::ArmModel::Pose& at, float size);

    class ToolGizmo
    {
    public:
        using Clock = PointerTarget::Clock;

        explicit ToolGizmo(const ui::Callback<void(const arm_host::ArmCommand& command)>& post);

        bool Grab(GizmoHandle handle, const SceneRay& ray, const arm_host::ArmSnapshot& snapshot, const ControlState& move, bool follow);
        void Drag(const SceneRay& ray, const arm_host::ArmSnapshot& snapshot, const ControlState& move, Clock::time_point now);
        void Release();
        void Cancel();
        bool Confirm(const ControlState& move);

        bool Dragging() const;
        bool Following() const;
        bool Reachable() const;
        std::optional<GizmoHandle> Handle() const;
        const std::optional<arm::JointPositions>& Target() const;
        std::optional<SceneNodes> TargetNodes() const;
        std::optional<ScenePoint> TargetPoint() const;
        std::string Label(const ControlState& move) const;
        ui::theme::StatusLevel Level() const;
        std::optional<arm::ArmModel::Pose> WantedPose() const;

    private:
        arm::ArmModel::Pose Wanted(float reading) const;

        PointerTarget pointer;
        std::optional<arm::ArmModel> model;
        arm::ArmGeometry geometry{};
        std::optional<GizmoHandle> handle;
        arm::JointPositions solution{};
        arm::ArmModel::Pose start{};
        arm::ArmModel::Pose wanted{};
        float startReading{ 0.0f };
        std::optional<arm::IkFailure> failure;
    };
}
