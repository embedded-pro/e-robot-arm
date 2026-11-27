#pragma once

#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ControlRules.hpp"
#include "ui/core/Callback.hpp"
#include <chrono>
#include <optional>
#include <string>

namespace simulator
{
    // The target a pointer drag in the 3-D view sets: Enter moves to it; on a simulation a follow drag moves live.
    class PointerTarget
    {
    public:
        using Clock = std::chrono::steady_clock;

        static constexpr Clock::duration followPeriod{ std::chrono::milliseconds{ 50 } };

        explicit PointerTarget(const ui::Callback<void(const arm_host::ArmCommand& command)>& post);

        static bool FollowOffered(const arm_host::ArmSnapshot& snapshot, const ControlState& move);

        void Begin(bool follow, const arm_host::ArmSnapshot& snapshot, const ControlState& move);
        bool KeepFollowing(const arm_host::ArmSnapshot& snapshot, const ControlState& move);
        void Set(const arm::JointPositions& joints, Clock::time_point now, bool sendable = true);
        void Release();
        void Cancel();
        bool Confirm(const ControlState& move);

        bool Dragging() const;
        bool Following() const;
        const std::optional<arm::JointPositions>& Target() const;
        std::string Suffix(const ControlState& move) const;

    private:
        void StopFollowing();

        ui::Callback<void(const arm_host::ArmCommand& command)> post;
        bool dragging{ false };
        bool following{ false };
        bool followRefused{ false };
        std::optional<arm::JointPositions> target;
        std::optional<Clock::time_point> lastFollow;
    };
}
