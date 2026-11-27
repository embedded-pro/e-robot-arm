#include "tools/simulator/view_models/ArmControllerTuningPresenter.hpp"
#include <cmath>
#include <optional>

namespace simulator
{
    namespace
    {
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupId;
        using ui::model::NumberTraits;
        using ui::model::OptionSpec;

        constexpr GroupId tuningGroup{ 0 };
        constexpr std::array<ui::model::GroupSpec, 1> groups{ { { tuningGroup, "Tuning" } } };
        constexpr std::array<ui::model::ActionSpec, 2> actions{ {
            { TuningAction::apply, "Apply", ui::theme::ButtonRole::Primary },
            { TuningAction::revert, "Revert" },
        } };
        constexpr std::array<OptionSpec, ArmControllerTuningPresenter::tunableCount> controllerOptions{ {
            { "gravity-pd" },
            { "computed-torque" },
            { "gravity-pid" },
            { "joint-impedance" },
            { "cartesian-impedance" },
            { "operational-space" },
            { "slotine-li" },
        } };
        constexpr std::array<std::string_view, arm::jointCount> stiffnessLabels{ "J1 stiffness", "J2 stiffness", "J3 stiffness", "J4 stiffness", "J5 stiffness",
            "J6 stiffness" };

        constexpr uint32_t Bit(arm::ArmControllerKind kind)
        {
            return 1u << static_cast<uint32_t>(kind);
        }

        using enum arm::ArmControllerKind;
        constexpr uint32_t bandwidthUsers{ Bit(gravityPd) | Bit(computedTorque) | Bit(gravityPid) | Bit(operationalSpace) | Bit(slotineLi) };
        constexpr uint32_t dampingUsers{ Bit(gravityPd) | Bit(computedTorque) | Bit(gravityPid) | Bit(jointImpedance) | Bit(cartesianImpedance) | Bit(operationalSpace) };

        FieldSpec Number(ui::model::FieldId id, std::string_view label, std::string_view suffix, NumberTraits traits, uint32_t users)
        {
            return { id, tuningGroup, FieldKind::Number, label, suffix, traits, {}, { TuningField::controller, users }, {} };
        }

        bool Same(float a, float b)
        {
            return std::abs(a - b) <= 1.0e-5f * std::max(1.0f, std::abs(b));
        }

        class Equal
        {
        public:
            bool operator()(const arm::BandwidthTuning& a, const arm::BandwidthTuning& b) const
            {
                return Same(a.bandwidth, b.bandwidth) && Same(a.dampingRatio, b.dampingRatio);
            }

            bool operator()(const arm::JointImpedanceTuning& a, const arm::JointImpedanceTuning& b) const
            {
                if (a.stiffness.has_value() != b.stiffness.has_value() || !Same(a.dampingRatio, b.dampingRatio))
                    return false;
                for (std::size_t joint = 0; a.stiffness.has_value() && joint != arm::jointCount; ++joint)
                    if (!Same((*a.stiffness)[joint], (*b.stiffness)[joint]))
                        return false;
                return true;
            }

            bool operator()(const arm::CartesianImpedanceTuning& a, const arm::CartesianImpedanceTuning& b) const
            {
                return Same(a.translationalStiffness, b.translationalStiffness) && Same(a.rotationalStiffness, b.rotationalStiffness) &&
                       Same(a.dampingRatio, b.dampingRatio);
            }

            bool operator()(const arm::OperationalSpaceTuning& a, const arm::OperationalSpaceTuning& b) const
            {
                return Same(a.bandwidth, b.bandwidth) && Same(a.dampingRatio, b.dampingRatio) && Same(a.singularityDamping, b.singularityDamping);
            }

            bool operator()(const arm::SlotineLiTuning& a, const arm::SlotineLiTuning& b) const
            {
                return Same(a.lambda, b.lambda) && Same(a.bandwidth, b.bandwidth) && Same(a.adaptationGain, b.adaptationGain);
            }

            template<class A, class B>
            bool operator()(const A&, const B&) const
            {
                return false;
            }
        };

        bool Matches(const arm::ArmControllerTuning& a, const arm::ArmControllerTuning& b)
        {
            return std::visit(Equal{}, a, b);
        }
    }

    ArmControllerTuningPresenter::Form::Form()
        : spec{ groups, fields, actions, {} }
        , model(spec, values, {})
    {
        using namespace arm::tuningLimit;
        fields[0] = { TuningField::controller, tuningGroup, FieldKind::Choice, "Controller", "", {}, controllerOptions, {}, {} };
        fields[1] = Number(TuningField::bandwidth, "Bandwidth", " rad/s", { 0.1, bandwidth, 1.0, 30.0, 1 }, bandwidthUsers);
        fields[2] = Number(TuningField::dampingRatio, "Damping ratio", "", { 0.01, dampingRatio, 0.05, 0.7, 2 }, dampingUsers);
        fields[3] = Number(TuningField::singularityDamping, "Singularity damping", "", { 0.001, singularityDamping, 0.005, 0.01, 3 }, Bit(operationalSpace));
        fields[4] = Number(TuningField::lambda, "Sliding gain λ", " 1/s", { 0.1, lambda, 1.0, 10.0, 1 }, Bit(slotineLi));
        fields[5] = Number(TuningField::adaptationGain, "Adaptation gain γ", "", { 0.1, adaptationGain, 1.0, 10.0, 1 }, Bit(slotineLi));
        fields[6] = Number(TuningField::translationalStiffness, "Translational stiffness", " N/m", { 0.1, 100000.0, 10.0, 50.0, 1 }, Bit(cartesianImpedance));
        fields[7] = Number(TuningField::rotationalStiffness, "Rotational stiffness", " N·m/rad", { 0.001, 1000.0, 0.01, 0.07, 3 }, Bit(cartesianImpedance));
        fields[8] = { TuningField::fromInertia, tuningGroup, FieldKind::Toggle, "Stiffness from inertia", "", {}, {}, { TuningField::controller, Bit(jointImpedance) }, {} };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            fields[TuningField::firstStiffness + joint] =
                Number(TuningField::Stiffness(joint), stiffnessLabels[joint], " N·m/rad", { 0.1, 100000.0, 10.0, 100.0, 1 }, Bit(jointImpedance));
        model.ResetToDefaults();
    }

    ArmControllerTuningPresenter::ArmControllerTuningPresenter(ui::shell::FormView& view, const ui::Callback<void(const arm_host::ArmCommand& command)>& post)
        : view(view)
        , post(post)
    {
        form.model.onActionTriggered = [this](ui::model::ActionId action)
        {
            OnAction(action);
        };
        view.Build(form.model);
        viewFieldChanged = form.model.onFieldChanged;
        form.model.onFieldChanged = [this](ui::model::FieldId field)
        {
            if (viewFieldChanged)
                viewFieldChanged(field);
            OnFieldChanged(field);
        };
        Load(arm::TuningOf(snapshot.config, Kind()));
        UpdateActions();
    }

    void ArmControllerTuningPresenter::Refresh(const arm_host::ArmSnapshot& newSnapshot, std::span<const arm_host::ArmEvent> events)
    {
        snapshot = newSnapshot;
        for (const auto& event : events)
            if (state == TuningState::requested && event.kind == arm_host::ArmEventKind::refused && event.commandName == "tune arm controller")
                SetState(TuningState::refused, event.reason);

        const auto inEffect = arm::TuningOf(snapshot.config, Kind());
        if (state == TuningState::applied)
            Load(inEffect);
        else if (state == TuningState::requested && Matches(FormTuning(), inEffect))
            SetState(TuningState::applied);
        UpdateActions();
    }

    void ArmControllerTuningPresenter::Choose(arm::ArmControllerKind kind)
    {
        if (!arm::Tunable(kind) || kind == Kind())
            return;
        form.model.SetSelection(TuningField::controller, static_cast<std::size_t>(kind));
        view.Refresh(TuningField::controller);
    }

    arm::ArmControllerKind ArmControllerTuningPresenter::Kind() const
    {
        return static_cast<arm::ArmControllerKind>(form.model.Selection(TuningField::controller));
    }

    arm::ArmControllerTuning ArmControllerTuningPresenter::FormTuning() const
    {
        const auto& model = form.model;
        switch (Kind())
        {
            case jointImpedance:
            {
                arm::JointImpedanceTuning tuning{ std::nullopt, model.Float(TuningField::dampingRatio) };
                if (!model.Flag(TuningField::fromInertia))
                {
                    arm::JointStiffnesses stiffness{};
                    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                        stiffness[joint] = model.Float(TuningField::Stiffness(joint));
                    tuning.stiffness = stiffness;
                }
                return tuning;
            }
            case cartesianImpedance:
                return arm::CartesianImpedanceTuning{ model.Float(TuningField::translationalStiffness), model.Float(TuningField::rotationalStiffness),
                    model.Float(TuningField::dampingRatio) };
            case operationalSpace:
                return arm::OperationalSpaceTuning{ model.Float(TuningField::bandwidth), model.Float(TuningField::dampingRatio), model.Float(TuningField::singularityDamping) };
            case slotineLi:
                return arm::SlotineLiTuning{ model.Float(TuningField::lambda), model.Float(TuningField::bandwidth), model.Float(TuningField::adaptationGain) };
            default:
                return arm::BandwidthTuning{ model.Float(TuningField::bandwidth), model.Float(TuningField::dampingRatio) };
        }
    }

    TuningState ArmControllerTuningPresenter::State() const
    {
        return state;
    }

    const std::string& ArmControllerTuningPresenter::StateText() const
    {
        return stateText;
    }

    ui::theme::StatusLevel ArmControllerTuningPresenter::StateLevel() const
    {
        return stateLevel;
    }

    const std::string& ArmControllerTuningPresenter::Hint() const
    {
        return hint;
    }

    void ArmControllerTuningPresenter::OnFieldChanged(ui::model::FieldId field)
    {
        if (loading)
            return;
        if (field == TuningField::controller)
        {
            SetState(TuningState::applied);
            Load(arm::TuningOf(snapshot.config, Kind()));
        }
        else
            SetState(Matches(FormTuning(), arm::TuningOf(snapshot.config, Kind())) ? TuningState::applied : TuningState::edited);
        UpdateActions();
    }

    void ArmControllerTuningPresenter::OnAction(ui::model::ActionId action)
    {
        if (action == TuningAction::revert)
        {
            SetState(TuningState::applied);
            Load(arm::TuningOf(snapshot.config, Kind()));
        }
        else if (ApplyRefusal().empty())
        {
            post(arm_host::TuneArmController{ Kind(), FormTuning() });
            SetState(TuningState::requested);
        }
        UpdateActions();
    }

    void ArmControllerTuningPresenter::Load(const arm::ArmControllerTuning& tuning)
    {
        loading = true;
        if (const auto* bandwidth = std::get_if<arm::BandwidthTuning>(&tuning))
            LoadBandwidth(*bandwidth);
        else if (const auto* impedance = std::get_if<arm::JointImpedanceTuning>(&tuning))
            LoadImpedance(*impedance);
        else if (const auto* cartesian = std::get_if<arm::CartesianImpedanceTuning>(&tuning))
        {
            form.model.SetNumber(TuningField::translationalStiffness, cartesian->translationalStiffness);
            form.model.SetNumber(TuningField::rotationalStiffness, cartesian->rotationalStiffness);
            form.model.SetNumber(TuningField::dampingRatio, cartesian->dampingRatio);
        }
        else if (const auto* space = std::get_if<arm::OperationalSpaceTuning>(&tuning))
        {
            LoadBandwidth({ space->bandwidth, space->dampingRatio });
            form.model.SetNumber(TuningField::singularityDamping, space->singularityDamping);
        }
        else if (const auto* adaptive = std::get_if<arm::SlotineLiTuning>(&tuning))
        {
            form.model.SetNumber(TuningField::lambda, adaptive->lambda);
            form.model.SetNumber(TuningField::bandwidth, adaptive->bandwidth);
            form.model.SetNumber(TuningField::adaptationGain, adaptive->adaptationGain);
        }
        loading = false;
        view.Refresh();
    }

    void ArmControllerTuningPresenter::LoadBandwidth(const arm::BandwidthTuning& tuning)
    {
        form.model.SetNumber(TuningField::bandwidth, tuning.bandwidth);
        form.model.SetNumber(TuningField::dampingRatio, tuning.dampingRatio);
    }

    void ArmControllerTuningPresenter::LoadImpedance(const arm::JointImpedanceTuning& tuning)
    {
        form.model.SetNumber(TuningField::dampingRatio, tuning.dampingRatio);
        form.model.SetFlag(TuningField::fromInertia, !tuning.stiffness.has_value());
        if (tuning.stiffness.has_value())
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                form.model.SetNumber(TuningField::Stiffness(joint), (*tuning.stiffness)[joint]);
    }

    void ArmControllerTuningPresenter::SetState(TuningState newState, std::string reason)
    {
        state = newState;
        switch (state)
        {
            case TuningState::applied:
                stateText = "Applied";
                stateLevel = ui::theme::StatusLevel::Ok;
                break;
            case TuningState::edited:
                stateText = "Edited";
                stateLevel = ui::theme::StatusLevel::Warning;
                break;
            case TuningState::requested:
                stateText = "Requested…";
                stateLevel = ui::theme::StatusLevel::Neutral;
                break;
            case TuningState::refused:
                stateText = "Refused: " + reason;
                stateLevel = ui::theme::StatusLevel::Fault;
                break;
        }
    }

    bool ArmControllerTuningPresenter::Offered() const
    {
        const auto name = arm::ArmControllerName(Kind());
        for (const auto& controller : snapshot.armControllers)
            if (controller.name == name)
                return true;
        return false;
    }

    std::string ArmControllerTuningPresenter::ApplyRefusal() const
    {
        if (!snapshot.targetReady)
            return "Not connected";
        if (!Offered())
            return "The arm does not offer this controller";
        if (snapshot.moving)
            return "Moving: wait or abort";
        if (state == TuningState::requested)
            return "Requested…";
        if (state == TuningState::applied)
            return "Nothing to apply";
        return {};
    }

    void ArmControllerTuningPresenter::UpdateActions()
    {
        const auto refusal = ApplyRefusal();
        view.SetActionEnabled(TuningAction::apply, refusal.empty());
        view.SetActionEnabled(TuningAction::revert, state == TuningState::edited || state == TuningState::refused);
        hint = refusal.empty() || state == TuningState::applied ? std::string{} : "Apply: " + refusal;
    }
}
