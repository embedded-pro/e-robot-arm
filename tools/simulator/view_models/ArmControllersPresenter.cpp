#include "tools/simulator/view_models/ArmControllersPresenter.hpp"
#include <format>

namespace simulator
{
    namespace
    {
        constexpr std::array<std::string_view, armControllerActionCount> actionNames{ "Select", "Enable", "Disable" };

        std::optional<arm::BandwidthTuning> TuningOf(const arm::ArmConfig& config, std::string_view name)
        {
            if (name == "gravity-pd")
                return config.gravityPd;
            if (name == "computed-torque")
                return config.computedTorque;
            if (name == "gravity-pid")
                return config.gravityPid;
            return std::nullopt;
        }

        std::string Parameters(const arm::ArmConfig& config, std::string_view name)
        {
            const auto tuning = TuningOf(config, name);
            if (!tuning.has_value())
                return "—";
            return std::format("ω {:.1f} rad/s, ζ {:.2f}", tuning->bandwidth, tuning->dampingRatio);
        }

        const char* StateText(const arm_host::ArmControllerStatus& controller)
        {
            if (controller.active)
                return "Active";
            return controller.enabled ? "Enabled" : "Disabled";
        }

        ArmControllerRow RowOf(const arm_host::ArmControllerStatus& controller, const arm::ArmConfig& config)
        {
            const auto* state = StateText(controller);
            const auto level = controller.active ? ui::theme::StatusLevel::Ok : ui::theme::StatusLevel::Neutral;
            return ArmControllerRow{ controller.name, state, level, Parameters(config, controller.name), controller.enabled };
        }

        ControlState Allowed()
        {
            return { true, {} };
        }

        ControlState Refused(std::string reason)
        {
            return { false, std::move(reason) };
        }
    }

    ArmControllersPresenter::ArmControllersPresenter(const ui::Callback<void(const arm_host::ArmCommand& command)>& post)
        : post(post)
    {
        Update();
    }

    void ArmControllersPresenter::Refresh(const arm_host::ArmSnapshot& newSnapshot)
    {
        snapshot = newSnapshot;
        rows.clear();
        for (const auto& controller : snapshot.armControllers)
            rows.push_back(RowOf(controller, snapshot.config));
        if (chosen.has_value() && *chosen >= rows.size())
            chosen.reset();
        Update();
    }

    void ArmControllersPresenter::Choose(std::optional<std::size_t> row)
    {
        chosen = row.has_value() && *row < rows.size() ? row : std::nullopt;
        Update();
    }

    void ArmControllersPresenter::Trigger(ArmControllerAction action)
    {
        if (!State(action).enabled)
            return;

        const auto id = static_cast<arm::ArmControllerId>(*chosen);
        switch (action)
        {
            case ArmControllerAction::select:
                return Post(arm_host::SelectArmController{ id });
            case ArmControllerAction::enable:
                return Post(arm_host::EnableArmController{ id });
            case ArmControllerAction::disable:
                if (snapshot.armControllers[*chosen].active && disablePrompt)
                    return disablePrompt(
                        std::format("Disable {}? The arm then has no controller and cannot be enabled until another one is selected.", rows[*chosen].name));
                return Post(arm_host::DisableArmController{ id });
        }
    }

    void ArmControllersPresenter::SetDisablePrompt(const ui::Callback<void(std::string_view prompt)>& prompt)
    {
        disablePrompt = prompt;
    }

    void ArmControllersPresenter::ConfirmDisable()
    {
        if (State(ArmControllerAction::disable).enabled)
            Post(arm_host::DisableArmController{ static_cast<arm::ArmControllerId>(*chosen) });
    }

    const std::vector<ArmControllerRow>& ArmControllersPresenter::Rows() const
    {
        return rows;
    }

    std::optional<std::size_t> ArmControllersPresenter::Chosen() const
    {
        return chosen;
    }

    const ControlState& ArmControllersPresenter::State(ArmControllerAction action) const
    {
        return states[static_cast<std::size_t>(action)];
    }

    const std::string& ArmControllersPresenter::Hint() const
    {
        return hint;
    }

    void ArmControllersPresenter::Update()
    {
        hint.clear();
        for (std::size_t action = 0; action != armControllerActionCount; ++action)
        {
            states[action] = Evaluate(static_cast<ArmControllerAction>(action));
            if (chosen.has_value() && !states[action].enabled && states[action].reason == "Disable the arm first")
                hint += std::format("{}{}: disable the arm first", hint.empty() ? "" : "\n", actionNames[action]);
        }
    }

    ControlState ArmControllersPresenter::Evaluate(ArmControllerAction action) const
    {
        if (!chosen.has_value())
            return Refused("Choose a controller");

        const auto& controller = snapshot.armControllers[*chosen];
        switch (action)
        {
            case ArmControllerAction::select:
                if (!controller.enabled)
                    return Refused("Enable it first");
                return controller.active ? Refused("Already active") : Allowed();
            case ArmControllerAction::enable:
                return controller.enabled ? Refused("Already enabled") : Allowed();
            case ArmControllerAction::disable:
                if (!controller.enabled)
                    return Refused("Already disabled");
                return controller.active && DrivesJoints(snapshot) ? Refused("Disable the arm first") : Allowed();
        }
        return Refused("Unknown action");
    }

    void ArmControllersPresenter::Post(const arm_host::ArmCommand& command) const
    {
        if (post)
            post(command);
    }
}
