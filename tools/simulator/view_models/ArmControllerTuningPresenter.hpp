#pragma once

#include "core/arm/ArmControllerTuning.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmEvent.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "ui/core/Callback.hpp"
#include "ui/model/FormModel.hpp"
#include "ui/shell/FormView.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <span>
#include <string>
#include <vector>

namespace simulator
{
    struct ControllerTuningField
    {
        static constexpr ui::model::FieldId controller{ 0 };
        static constexpr ui::model::FieldId bandwidth{ 1 };
        static constexpr ui::model::FieldId dampingRatio{ 2 };
        static constexpr ui::model::FieldId singularityDamping{ 3 };
        static constexpr ui::model::FieldId lambda{ 4 };
        static constexpr ui::model::FieldId adaptationGain{ 5 };
        static constexpr ui::model::FieldId translationalStiffness{ 6 };
        static constexpr ui::model::FieldId rotationalStiffness{ 7 };
        static constexpr ui::model::FieldId fromInertia{ 8 };
        static constexpr uint16_t firstStiffness{ 9 };

        static constexpr ui::model::FieldId Stiffness(std::size_t joint)
        {
            return ui::model::FieldId{ static_cast<uint16_t>(firstStiffness + joint) };
        }
    };

    struct ControllerTuningAction
    {
        static constexpr ui::model::ActionId apply{ 0 };
        static constexpr ui::model::ActionId revert{ 1 };
    };

    enum class ControllerTuningState : uint8_t
    {
        applied,
        edited,
        requested,
        refused
    };

    class ArmControllerTuningPresenter
    {
    public:
        static constexpr std::size_t tunableCount{ 7 };
        static constexpr std::size_t fieldCount{ ControllerTuningField::firstStiffness + arm::jointCount };

        ArmControllerTuningPresenter(ui::shell::FormView& view, const ui::Callback<void(const arm_host::ArmCommand& command)>& post);
        ArmControllerTuningPresenter(const ArmControllerTuningPresenter& other) = delete;
        ArmControllerTuningPresenter& operator=(const ArmControllerTuningPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events);
        void Choose(arm::ArmControllerKind kind);

        arm::ArmControllerKind Kind() const;
        arm::ArmControllerTuning FormTuning() const;
        ControllerTuningState State() const;
        const std::string& StateText() const;
        ui::theme::StatusLevel StateLevel() const;
        const std::string& Hint() const;

        std::vector<std::string> Presets() const;
        std::string LoadRefusal(std::size_t preset) const;
        void LoadPreset(std::size_t preset);
        std::string SaveRefusal(const std::string& name) const;
        bool Replaces(const std::string& name) const;
        void SavePreset(const std::string& name);
        const std::string& PresetNote() const;

    private:
        struct Form
        {
            Form();
            Form(const Form& other) = delete;
            Form& operator=(const Form& other) = delete;

            std::array<ui::model::FieldSpec, fieldCount> fields;
            ui::model::FormSpec spec;
            std::array<ui::model::FieldValue, fieldCount> values{};
            ui::model::FormModel model;
        };

        void OnFieldChanged(ui::model::FieldId field);
        void OnAction(ui::model::ActionId action);
        void Load(const arm::ArmControllerTuning& tuning);
        void LoadItem(const arm::BandwidthTuning& tuning);
        void LoadItem(const arm::JointImpedanceTuning& tuning);
        void LoadItem(const arm::CartesianImpedanceTuning& tuning);
        void LoadItem(const arm::OperationalSpaceTuning& tuning);
        void LoadItem(const arm::SlotineLiTuning& tuning);
        void SetState(ControllerTuningState newState, const std::string& reason = {});
        bool Offered() const;
        const arm::ArmControllerPreset* Preset(std::size_t index) const;
        std::string ApplyRefusal() const;
        void UpdateActions();

        ui::shell::FormView& view;
        ui::Callback<void(const arm_host::ArmCommand& command)> post;
        Form form;
        ui::Callback<void(ui::model::FieldId)> viewFieldChanged;
        arm_host::ArmSnapshot snapshot;
        ControllerTuningState state{ ControllerTuningState::applied };
        std::string stateText{ "Applied" };
        ui::theme::StatusLevel stateLevel{ ui::theme::StatusLevel::Ok };
        std::string hint;
        std::string presetNote;
        bool loading{ false };
    };
}
