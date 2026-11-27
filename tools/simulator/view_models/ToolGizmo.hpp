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
#include <cstdint>
#include <optional>
#include <string>

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
