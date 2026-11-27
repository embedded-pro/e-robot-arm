#include "tools/simulator/view_models/PointerTarget.hpp"

namespace simulator
{
    PointerTarget::PointerTarget(const ui::Callback<void(const arm_host::ArmCommand& command)>& post)
        : post(post)
    {}

    bool PointerTarget::FollowOffered(const arm_host::ArmSnapshot& snapshot, const ControlState& move)
    {
        return move.enabled && snapshot.disturbances.has_value();
    }

    void PointerTarget::Begin(bool follow, const arm_host::ArmSnapshot& snapshot, const ControlState& move)
    {
        StopFollowing();
        following = follow && FollowOffered(snapshot, move);
        followRefused = follow && !following;
        dragging = true;
        target.reset();
        lastFollow.reset();
    }

    bool PointerTarget::KeepFollowing(const arm_host::ArmSnapshot& snapshot, const ControlState& move)
    {
        if (!following || FollowOffered(snapshot, move))
            return true;
        Release();
        return false;
    }

    void PointerTarget::Set(const arm::JointPositions& joints, Clock::time_point now, bool sendable)
    {
        target = joints;
        if (!following || !sendable || (lastFollow.has_value() && now - *lastFollow < followPeriod))
            return;
        lastFollow = now;
        post(arm_host::MoveTo{ joints });
    }

    void PointerTarget::Release()
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

    void PointerTarget::Cancel()
    {
        Release();
        target.reset();
    }

    bool PointerTarget::Confirm(const ControlState& move)
    {
        if (dragging || !target.has_value() || !move.enabled)
            return false;
        post(arm_host::MoveTo{ *target });
        target.reset();
        return true;
    }

    bool PointerTarget::Dragging() const
    {
        return dragging;
    }

    bool PointerTarget::Following() const
    {
        return following;
    }

    const std::optional<arm::JointPositions>& PointerTarget::Target() const
    {
        return target;
    }

    std::string PointerTarget::Suffix(const ControlState& move) const
    {
        if (following)
            return " · following";
        const std::string refused = followRefused ? " · follow: simulation only" : "";
        if (move.enabled)
            return refused + " · Enter: move · Del: clear";
        return refused + " · Move: " + move.reason;
    }

    void PointerTarget::StopFollowing()
    {
        if (following)
            post(arm_host::AbortMove{});
        following = false;
    }
}
