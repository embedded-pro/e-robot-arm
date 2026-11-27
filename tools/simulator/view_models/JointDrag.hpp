#pragma once

#include "core/arm_model/ArmModel.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ControlRules.hpp"
#include "tools/simulator/view_models/ScenePicking.hpp"
#include "ui/core/Callback.hpp"
#include <chrono>
#include <cstddef>
#include <optional>
#include <string>

namespace simulator
{
    class JointDrag
    {
    public:
        using Clock = std::chrono::steady_clock;

        static constexpr std::size_t solverSteps{ 8 };
        static constexpr Clock::duration followPeriod{ std::chrono::milliseconds{ 50 } };

        explicit JointDrag(const ui::Callback<void(const arm_host::ArmCommand& command)>& post);

        static bool FollowOffered(const arm_host::ArmSnapshot& snapshot, const ControlState& move);

        bool Grab(int pickedJoint, const ScenePoint& hit, const arm_host::ArmSnapshot& snapshot, const ControlState& move, bool follow);
        void Drag(const SceneRay& ray, const arm_host::ArmSnapshot& snapshot, const ControlState& move, Clock::time_point now);
        void Release();
        void Cancel();
        bool Confirm(const ControlState& move);

        bool Dragging() const;
        bool Following() const;
        const std::optional<arm::JointPositions>& Target() const;
        std::string Label(const ControlState& move) const;

    private:
        math::Vector3<float> PointAt(float angle) const;
        float Solve(const SceneRay& ray, float angle);
        void StopFollowing();

        ui::Callback<void(const arm_host::ArmCommand& command)> post;
        std::optional<arm::ArmModel> model;
        std::optional<std::size_t> joint;
        bool dragging{ false };
        bool following{ false };
        bool followRefused{ false };
        bool atLimit{ false };
        arm::JointPositions start{};
        math::Vector3<float> local{};
        foc::Radians lower{};
        foc::Radians upper{};
        std::optional<arm::JointPositions> target;
        std::optional<Clock::time_point> lastFollow;
    };
}
