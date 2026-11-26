#include "tools/simulator/view_models/JogPresenter.hpp"
#include <algorithm>
#include <format>

namespace simulator
{
    namespace
    {
        constexpr std::array<const char*, jogAxisCount> toolAxisNames{ "X", "Y", "Z", "Rx", "Ry", "Rz" };

        bool Rotational(std::size_t axis)
        {
            return axis >= 3;
        }
    }

    JogPresenter::JogPresenter(const ui::Callback<void(const arm_host::ArmCommand& command)>& post)
        : post(post)
    {
        Update();
    }

    void JogPresenter::Refresh(const arm_host::ArmSnapshot& newSnapshot, Clock::time_point now)
    {
        snapshot = newSnapshot;
        Update();
        if (!hold.has_value())
            return;
        if (!jogState.enabled)
        {
            hold.reset();
            Update();
            return;
        }
        if (now - hold->sent >= RefreshPeriod())
        {
            Send(hold->axis, static_cast<float>(hold->direction) * Speed(hold->axis));
            hold->sent = now;
        }
    }

    void JogPresenter::SetSpace(JogSpace newSpace)
    {
        if (hold.has_value())
            return;
        space = newSpace;
        Update();
    }

    void JogPresenter::SetShare(int percent)
    {
        share = std::clamp(percent, 1, 100);
        Update();
    }

    void JogPresenter::ChooseAxis(std::size_t axis)
    {
        if (axis < jogAxisCount && !hold.has_value())
            chosen = axis;
    }

    void JogPresenter::Press(std::size_t axis, int direction, Clock::time_point now)
    {
        if (axis >= jogAxisCount || direction == 0 || !jogState.enabled || hold.has_value())
            return;
        chosen = axis;
        hold = Hold{ axis, direction > 0 ? 1 : -1, now };
        Send(axis, static_cast<float>(hold->direction) * Speed(axis));
        Update();
    }

    void JogPresenter::Release()
    {
        if (!hold.has_value())
            return;
        Send(hold->axis, 0.0f);
        hold.reset();
        Update();
    }

    JogSpace JogPresenter::Space() const
    {
        return space;
    }

    int JogPresenter::Share() const
    {
        return share;
    }

    std::optional<std::size_t> JogPresenter::ChosenAxis() const
    {
        return chosen;
    }

    bool JogPresenter::Held() const
    {
        return hold.has_value();
    }

    const ControlState& JogPresenter::JogState() const
    {
        return jogState;
    }

    const ControlState& JogPresenter::SpaceState() const
    {
        return spaceState;
    }

    const std::array<JogRow, jogAxisCount>& JogPresenter::Rows() const
    {
        return rows;
    }

    const std::string& JogPresenter::Status() const
    {
        return status;
    }

    void JogPresenter::Update()
    {
        jogState = Evaluate();
        spaceState = hold.has_value() ? ControlState{ false, "Release the jog first" } : ControlState{ true, {} };
        for (std::size_t axis = 0; axis != jogAxisCount; ++axis)
        {
            const auto unit = space == JogSpace::joints || Rotational(axis) ? "rad/s" : "m/s";
            const auto label = space == JogSpace::joints ? std::format("J{}", axis + 1) : std::string{ toolAxisNames[axis] };
            rows[axis] = JogRow{ label, std::format("{:.3f} {}", Speed(axis), unit) };
        }
        status.clear();
        if (hold.has_value())
        {
            const auto* sign = hold->direction < 0 ? "−" : "+";
            status = std::format("Jogging {} {}{}", rows[hold->axis].label, sign, rows[hold->axis].speed);
        }
    }

    ControlState JogPresenter::Evaluate() const
    {
        using enum arm::ArmState;
        if (!snapshot.targetReady)
            return { false, "Not connected" };
        if (snapshot.state == tripped)
            return { false, "Tripped" };
        if (snapshot.state == holding)
            return { false, "Holding: re-arm first" };
        if (snapshot.state == moving && !hold.has_value())
            return { false, "Moving: wait or abort" };
        if (snapshot.state != enabled && snapshot.state != moving)
            return { false, "Enable the arm first" };
        if (arm_host::FreeMode(snapshot))
            return { false, "Free mode: select a tracking controller" };
        return { true, {} };
    }

    float JogPresenter::Speed(std::size_t axis) const
    {
        const auto fraction = static_cast<float>(share) / 100.0f;
        if (space == JogSpace::joints)
            return fraction * snapshot.config.motionLimits[axis].maxSpeed.Value();
        const auto& limits = snapshot.config.toolLimits;
        return fraction * (Rotational(axis) ? limits.angularSpeedRadPerS : limits.speedMPerS);
    }

    JogPresenter::Clock::duration JogPresenter::RefreshPeriod() const
    {
        return std::chrono::duration_cast<Clock::duration>(snapshot.config.controlPeriod * arm::jogDeadManPeriods) / 2;
    }

    void JogPresenter::Send(std::size_t axis, float speed) const
    {
        if (!post)
            return;
        if (space == JogSpace::joints)
            post(arm_host::JogJoint{ axis, foc::RadiansPerSecond{ speed } });
        else
            post(arm_host::JogTool{ arm::ToolJog{ space == JogSpace::base ? arm::JogFrame::base : arm::JogFrame::tool, static_cast<arm::ToolAxis>(axis), speed } });
    }
}
