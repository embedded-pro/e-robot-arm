#include "tools/simulator/view_models/DisturbancePresenter.hpp"
#include <cmath>
#include <format>

namespace simulator
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::FieldKind;
        using ui::model::OptionSpec;
        using ui::theme::ButtonRole;

        constexpr float maximumTorque{ 200.0f };

        constexpr std::array<OptionSpec, arm::jointCount> jointOptions{ { { "J1" }, { "J2" }, { "J3" }, { "J4" }, { "J5" }, { "J6" } } };

        constexpr std::array<ui::model::FieldSpec, 2> fields{ {
            { DisturbanceField::joint, ui::model::noGroup, FieldKind::Choice, "Joint", "", {}, jointOptions, {}, {} },
            { DisturbanceField::torque, ui::model::noGroup, FieldKind::Number, "Collision torque", "N·m", { -maximumTorque, maximumTorque, 0.5, 30.0, 2 }, {}, {},
                {} },
        } };

        constexpr std::array<ActionSpec, disturbanceActionCount> actions{ {
            { DisturbanceAction::injectFault, "Inject Fault", ButtonRole::Stop },
            { DisturbanceAction::silence, "Silence" },
            { DisturbanceAction::resume, "Resume" },
            { DisturbanceAction::applyTorque, "Apply Torque" },
            { DisturbanceAction::clearTorque, "Clear Torque", ButtonRole::Reset },
            { DisturbanceAction::powerOff, "Power Off", ButtonRole::Stop },
        } };

        bool MotorUp(const arm_host::ArmSnapshot& snapshot, std::size_t joint)
        {
            return snapshot.virtualMotors.has_value() && snapshot.virtualMotors->motors[joint].phase == virtual_arm::MotorPhase::up;
        }
    }

    DisturbancePresenter::Form::Form()
        : spec{ {}, fields, actions, {} }
        , model(spec, values, {})
    {
        model.ResetToDefaults();
    }

    DisturbancePresenter::DisturbancePresenter(ui::shell::FormView& view, const ui::Callback<void(const arm_host::ArmCommand& command)>& post)
        : view(view)
        , post(post)
    {
        form.model.onActionTriggered = [this](ui::model::ActionId action)
        {
            OnAction(action);
        };
        view.Build(form.model);
        Update();
    }

    void DisturbancePresenter::Refresh(const arm_host::ArmSnapshot& newSnapshot)
    {
        snapshot = newSnapshot;
        Update();
    }

    void DisturbancePresenter::SelectJoint(std::size_t joint)
    {
        if (joint >= arm::jointCount || joint == Joint())
            return;

        form.model.SetSelection(DisturbanceField::joint, joint);
        view.Refresh(DisturbanceField::joint);
        Update();
    }

    bool DisturbancePresenter::Visible() const
    {
        return snapshot.disturbances.has_value();
    }

    std::size_t DisturbancePresenter::Joint() const
    {
        return form.model.Selection(DisturbanceField::joint);
    }

    bool DisturbancePresenter::Enabled(ui::model::ActionId action) const
    {
        return refusals[action.value].empty();
    }

    const std::string& DisturbancePresenter::Active() const
    {
        return active;
    }

    const std::string& DisturbancePresenter::Hint() const
    {
        return hint;
    }

    void DisturbancePresenter::OnAction(ui::model::ActionId action)
    {
        Update();
        if (!Enabled(action))
            return;

        const auto joint = Joint();
        if (action == DisturbanceAction::injectFault)
            post(arm_host::InjectFault{ joint, can::FocFaultCode::overCurrent });
        else if (action == DisturbanceAction::silence)
            post(arm_host::SilenceJoint{ joint });
        else if (action == DisturbanceAction::resume)
            post(arm_host::ResumeJoint{ joint });
        else if (action == DisturbanceAction::applyTorque)
            post(arm_host::CollideJoint{ joint, foc::NewtonMeter{ static_cast<float>(form.model.Number(DisturbanceField::torque)) } });
        else if (action == DisturbanceAction::clearTorque)
            post(arm_host::CollideJoint{ joint, foc::NewtonMeter{ 0.0f } });
        else
            post(arm_host::PowerOffMotor{ joint });
    }

    std::string DisturbancePresenter::OfferRefusal(ui::model::ActionId action) const
    {
        if (!snapshot.disturbances.has_value())
            return "not a simulation";
        if (!snapshot.targetReady)
            return "not connected";

        const auto& offered = *snapshot.disturbances;
        if (action == DisturbanceAction::injectFault && !offered.injectFault)
            return "not on the virtual arm";
        if ((action == DisturbanceAction::applyTorque || action == DisturbanceAction::clearTorque) && !offered.collision)
            return "the simulated joints have no dynamics";
        if (action == DisturbanceAction::powerOff && !offered.powerOff)
            return "not on simulated joints";
        return {};
    }

    std::string DisturbancePresenter::Refusal(ui::model::ActionId action) const
    {
        if (auto refusal = OfferRefusal(action); !refusal.empty())
            return refusal;

        const auto joint = Joint();
        const auto& applied = *snapshot.disturbances;
        if (action == DisturbanceAction::silence && applied.silent[joint])
            return std::format("J{} is silent", joint + 1);
        if (action == DisturbanceAction::resume && !applied.silent[joint])
            return std::format("J{} is answering", joint + 1);
        if (action == DisturbanceAction::clearTorque && applied.torque[joint].Value() == 0.0f)
            return std::format("no torque on J{}", joint + 1);
        if (action == DisturbanceAction::powerOff && !MotorUp(snapshot, joint))
            return std::format("J{}'s motor is not up", joint + 1);
        return {};
    }

    void DisturbancePresenter::Update()
    {
        hint.clear();
        for (std::size_t index = 0; index != disturbanceActionCount; ++index)
        {
            const auto action = actions[index].id;
            refusals[index] = Refusal(action);
            view.SetActionEnabled(action, refusals[index].empty());
            if (!refusals[index].empty())
                hint += std::format("{}{}: {}", hint.empty() ? "" : "\n", actions[index].label, refusals[index]);
        }

        active.clear();
        if (snapshot.disturbances.has_value())
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                if (snapshot.disturbances->silent[joint])
                    active += std::format("{}J{} silent", active.empty() ? "" : " · ", joint + 1);
                if (const auto torque = snapshot.disturbances->torque[joint].Value(); torque != 0.0f)
                    active += std::format("{}J{} {:.2f} N·m", active.empty() ? "" : " · ", joint + 1, torque);
            }
        active = "Active: " + (active.empty() ? std::string{ "—" } : active);
    }
}
