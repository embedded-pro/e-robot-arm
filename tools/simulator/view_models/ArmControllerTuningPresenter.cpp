#include "tools/simulator/view_models/ArmControllerTuningPresenter.hpp"
#include <algorithm>
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
            { ControllerTuningAction::apply, "Apply", ui::theme::ButtonRole::Primary },
            { ControllerTuningAction::revert, "Revert" },
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

        FieldSpec Number(ui::model::FieldId id, std::string_view label, std::string_view suffix, const NumberTraits& traits, uint32_t users)
        {
            return { id, tuningGroup, FieldKind::Number, label, suffix, traits, {}, { ControllerTuningField::controller, users }, {} };
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
        fields[0] = { ControllerTuningField::controller, tuningGroup, FieldKind::Choice, "Controller", "", {}, controllerOptions, {}, {} };
        fields[1] = Number(ControllerTuningField::bandwidth, "Bandwidth", " rad/s", { 0.1, bandwidth, 1.0, 30.0, 1 }, bandwidthUsers);
        fields[2] = Number(ControllerTuningField::dampingRatio, "Damping ratio", "", { 0.01, dampingRatio, 0.05, 0.7, 2 }, dampingUsers);
        fields[3] = Number(ControllerTuningField::singularityDamping, "Singularity damping", "", { 0.001, singularityDamping, 0.005, 0.01, 3 }, Bit(operationalSpace));
        fields[4] = Number(ControllerTuningField::lambda, "Sliding gain λ", " 1/s", { 0.1, lambda, 1.0, 10.0, 1 }, Bit(slotineLi));
        fields[5] = Number(ControllerTuningField::adaptationGain, "Adaptation gain γ", "", { 0.1, adaptationGain, 1.0, 10.0, 1 }, Bit(slotineLi));
        fields[6] = Number(ControllerTuningField::translationalStiffness, "Translational stiffness", " N/m", { 0.1, 100000.0, 10.0, 50.0, 1 }, Bit(cartesianImpedance));
        fields[7] = Number(ControllerTuningField::rotationalStiffness, "Rotational stiffness", " N·m/rad", { 0.001, 1000.0, 0.01, 0.07, 3 }, Bit(cartesianImpedance));
        fields[8] = { ControllerTuningField::fromInertia, tuningGroup, FieldKind::Toggle, "Stiffness from inertia", "", {}, {}, { ControllerTuningField::controller, Bit(jointImpedance) }, {} };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            fields[ControllerTuningField::firstStiffness + joint] =
                Number(ControllerTuningField::Stiffness(joint), stiffnessLabels[joint], " N·m/rad", { 0.1, 100000.0, 10.0, 100.0, 1 }, Bit(jointImpedance));
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
            if (state == ControllerTuningState::requested && event.kind == arm_host::ArmEventKind::refused && event.commandName == "tune arm controller")
                SetState(ControllerTuningState::refused, event.reason);

        const auto inEffect = arm::TuningOf(snapshot.config, Kind());
        if (state == ControllerTuningState::applied)
            Load(inEffect);
        else if (state == ControllerTuningState::requested && Matches(FormTuning(), inEffect))
            SetState(ControllerTuningState::applied);
        UpdateActions();
    }

    void ArmControllerTuningPresenter::Choose(arm::ArmControllerKind kind)
    {
        if (!arm::Tunable(kind) || kind == Kind())
            return;
        presetNote.clear();
        form.model.SetSelection(ControllerTuningField::controller, static_cast<std::size_t>(kind));
        view.Refresh(ControllerTuningField::controller);
    }

    arm::ArmControllerKind ArmControllerTuningPresenter::Kind() const
    {
        return static_cast<arm::ArmControllerKind>(form.model.Selection(ControllerTuningField::controller));
    }

    arm::ArmControllerTuning ArmControllerTuningPresenter::FormTuning() const
    {
        const auto& model = form.model;
        switch (Kind())
        {
            case jointImpedance:
            {
                arm::JointImpedanceTuning tuning{ std::nullopt, model.Float(ControllerTuningField::dampingRatio) };
                if (!model.Flag(ControllerTuningField::fromInertia))
                {
                    arm::JointStiffnesses stiffness{};
                    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                        stiffness[joint] = model.Float(ControllerTuningField::Stiffness(joint));
                    tuning.stiffness = stiffness;
                }
                return tuning;
            }
            case cartesianImpedance:
                return arm::CartesianImpedanceTuning{ model.Float(ControllerTuningField::translationalStiffness), model.Float(ControllerTuningField::rotationalStiffness),
                    model.Float(ControllerTuningField::dampingRatio) };
            case operationalSpace:
                return arm::OperationalSpaceTuning{ model.Float(ControllerTuningField::bandwidth), model.Float(ControllerTuningField::dampingRatio), model.Float(ControllerTuningField::singularityDamping) };
            case slotineLi:
                return arm::SlotineLiTuning{ model.Float(ControllerTuningField::lambda), model.Float(ControllerTuningField::bandwidth), model.Float(ControllerTuningField::adaptationGain) };
            default:
                return arm::BandwidthTuning{ model.Float(ControllerTuningField::bandwidth), model.Float(ControllerTuningField::dampingRatio) };
        }
    }

    ControllerTuningState ArmControllerTuningPresenter::State() const
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

    std::vector<std::string> ArmControllerTuningPresenter::Presets() const
    {
        std::vector<std::string> names;
        for (std::size_t index = 0; index != snapshot.config.presetCount; ++index)
            if (snapshot.config.presets[index].controller == Kind())
                names.emplace_back(snapshot.config.presets[index].name.begin(), snapshot.config.presets[index].name.end());
        return names;
    }

    std::string ArmControllerTuningPresenter::LoadRefusal(std::size_t preset) const
    {
        if (Preset(preset) == nullptr)
            return "Choose a preset";
        if (!snapshot.targetReady)
            return "Not connected";
        if (!Offered())
            return "The arm does not offer this controller";
        if (snapshot.moving)
            return "Moving: wait or abort";
        if (state == ControllerTuningState::requested)
            return "Requested…";
        return {};
    }

    void ArmControllerTuningPresenter::LoadPreset(std::size_t preset)
    {
        if (const auto refusal = LoadRefusal(preset); !refusal.empty())
        {
            presetNote = "Load: " + refusal;
            return;
        }
        presetNote = "Loaded “" + std::string{ Preset(preset)->name.begin(), Preset(preset)->name.end() } + "”";
        Load(Preset(preset)->tuning);
        post(arm_host::TuneArmController{ Kind(), FormTuning() });
        SetState(ControllerTuningState::requested);
        UpdateActions();
    }

    std::string ArmControllerTuningPresenter::SaveRefusal(const std::string& name) const
    {
        if (!snapshot.targetReady)
            return "Not connected";
        if (name.empty() || name.size() > arm::presetNameSize)
            return "Name it (1–16 characters)";
        if (!Replaces(name) && snapshot.config.presetCount == arm::maxArmControllerPresets)
            return "16 presets already: replace one";
        return {};
    }

    bool ArmControllerTuningPresenter::Replaces(const std::string& name) const
    {
        const auto names = Presets();
        return std::ranges::find(names, name) != names.end();
    }

    void ArmControllerTuningPresenter::SavePreset(const std::string& name)
    {
        if (const auto refusal = SaveRefusal(name); !refusal.empty())
        {
            presetNote = "Save: " + refusal;
            return;
        }
        presetNote = (Replaces(name) ? "Replaced “" : "Saved “") + name + "” with the applied tuning";
        post(arm_host::SaveArmControllerPreset{ Kind(), name });
    }

    const std::string& ArmControllerTuningPresenter::PresetNote() const
    {
        return presetNote;
    }

    const arm::ArmControllerPreset* ArmControllerTuningPresenter::Preset(std::size_t index) const
    {
        std::size_t seen{ 0 };
        for (std::size_t slot = 0; slot != snapshot.config.presetCount; ++slot)
        {
            if (snapshot.config.presets[slot].controller != Kind())
                continue;
            if (seen == index)
                return &snapshot.config.presets[slot];
            ++seen;
        }
        return nullptr;
    }

    void ArmControllerTuningPresenter::OnFieldChanged(ui::model::FieldId field)
    {
        if (loading)
            return;
        if (field == ControllerTuningField::controller)
        {
            SetState(ControllerTuningState::applied);
            Load(arm::TuningOf(snapshot.config, Kind()));
        }
        else
            SetState(Matches(FormTuning(), arm::TuningOf(snapshot.config, Kind())) ? ControllerTuningState::applied : ControllerTuningState::edited);
        UpdateActions();
    }

    void ArmControllerTuningPresenter::OnAction(ui::model::ActionId action)
    {
        if (action == ControllerTuningAction::revert)
        {
            SetState(ControllerTuningState::applied);
            Load(arm::TuningOf(snapshot.config, Kind()));
        }
        else if (ApplyRefusal().empty())
        {
            post(arm_host::TuneArmController{ Kind(), FormTuning() });
            SetState(ControllerTuningState::requested);
        }
        UpdateActions();
    }

    void ArmControllerTuningPresenter::Load(const arm::ArmControllerTuning& tuning)
    {
        loading = true;
        std::visit([this](const auto& item)
            {
                LoadItem(item);
            },
            tuning);
        loading = false;
        view.Refresh();
    }

    void ArmControllerTuningPresenter::LoadItem(const arm::BandwidthTuning& tuning)
    {
        form.model.SetNumber(ControllerTuningField::bandwidth, tuning.bandwidth);
        form.model.SetNumber(ControllerTuningField::dampingRatio, tuning.dampingRatio);
    }

    void ArmControllerTuningPresenter::LoadItem(const arm::JointImpedanceTuning& tuning)
    {
        form.model.SetNumber(ControllerTuningField::dampingRatio, tuning.dampingRatio);
        form.model.SetFlag(ControllerTuningField::fromInertia, !tuning.stiffness.has_value());
        if (tuning.stiffness.has_value())
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                form.model.SetNumber(ControllerTuningField::Stiffness(joint), (*tuning.stiffness)[joint]);
    }

    void ArmControllerTuningPresenter::LoadItem(const arm::CartesianImpedanceTuning& tuning)
    {
        form.model.SetNumber(ControllerTuningField::translationalStiffness, tuning.translationalStiffness);
        form.model.SetNumber(ControllerTuningField::rotationalStiffness, tuning.rotationalStiffness);
        form.model.SetNumber(ControllerTuningField::dampingRatio, tuning.dampingRatio);
    }

    void ArmControllerTuningPresenter::LoadItem(const arm::OperationalSpaceTuning& tuning)
    {
        LoadItem(arm::BandwidthTuning{ tuning.bandwidth, tuning.dampingRatio });
        form.model.SetNumber(ControllerTuningField::singularityDamping, tuning.singularityDamping);
    }

    void ArmControllerTuningPresenter::LoadItem(const arm::SlotineLiTuning& tuning)
    {
        form.model.SetNumber(ControllerTuningField::lambda, tuning.lambda);
        form.model.SetNumber(ControllerTuningField::bandwidth, tuning.bandwidth);
        form.model.SetNumber(ControllerTuningField::adaptationGain, tuning.adaptationGain);
    }

    void ArmControllerTuningPresenter::SetState(ControllerTuningState newState, const std::string& reason)
    {
        state = newState;
        switch (state)
        {
            case ControllerTuningState::applied:
                stateText = "Applied";
                stateLevel = ui::theme::StatusLevel::Ok;
                break;
            case ControllerTuningState::edited:
                stateText = "Edited";
                stateLevel = ui::theme::StatusLevel::Warning;
                break;
            case ControllerTuningState::requested:
                stateText = "Requested…";
                stateLevel = ui::theme::StatusLevel::Neutral;
                break;
            case ControllerTuningState::refused:
                stateText = "Refused: " + reason;
                stateLevel = ui::theme::StatusLevel::Fault;
                break;
        }
    }

    bool ArmControllerTuningPresenter::Offered() const
    {
        const std::string_view name{ arm::ArmControllerName(Kind()) };
        return std::ranges::any_of(snapshot.armControllers, [name](const arm_host::ArmControllerStatus& controller)
            {
                return controller.name == name;
            });
    }

    std::string ArmControllerTuningPresenter::ApplyRefusal() const
    {
        if (!snapshot.targetReady)
            return "Not connected";
        if (!Offered())
            return "The arm does not offer this controller";
        if (snapshot.moving)
            return "Moving: wait or abort";
        if (state == ControllerTuningState::requested)
            return "Requested…";
        if (state == ControllerTuningState::applied)
            return "Nothing to apply";
        return {};
    }

    void ArmControllerTuningPresenter::UpdateActions()
    {
        const auto refusal = ApplyRefusal();
        view.SetActionEnabled(ControllerTuningAction::apply, refusal.empty());
        view.SetActionEnabled(ControllerTuningAction::revert, state == ControllerTuningState::edited || state == ControllerTuningState::refused);
        hint = refusal.empty() || state == ControllerTuningState::applied ? std::string{} : "Apply: " + refusal;
    }
}
