#ifndef TOOLS_SIMULATOR_VIEW_MODELS_SIMULATOR_PRESENTER_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_SIMULATOR_PRESENTER_HPP

#include "tools/arm_host/ArmCommands.hpp"
#include "tools/simulator/view_models/ArmSceneModel.hpp"
#include "tools/simulator/view_models/ControlRules.hpp"
#include "tools/simulator/view_models/EventLog.hpp"
#include "tools/simulator/view_models/JointRows.hpp"
#include "tools/simulator/view_models/ToolReadout.hpp"
#include "ui/core/Callback.hpp"
#include "ui/model/FormModel.hpp"
#include "ui/shell/AppShell.hpp"
#include "ui/shell/FormView.hpp"
#include <array>
#include <map>
#include <span>
#include <string>

namespace simulator
{
    struct StatusStrip
    {
        std::string target;
        std::string connection;
        std::string stage;
        ui::theme::StatusLevel stageLevel{ ui::theme::StatusLevel::Neutral };
        ui::theme::StatusLevel connectionLevel{ ui::theme::StatusLevel::Neutral };
        std::string contracts;
        std::string trip;
        ui::theme::StatusLevel tripLevel{ ui::theme::StatusLevel::Neutral };
        std::string overruns;
        ui::theme::StatusLevel overrunLevel{ ui::theme::StatusLevel::Neutral };
        bool realArm{ false };
    };

    struct OperateAction
    {
        static constexpr ui::model::ActionId bringUp{ 0 };
        static constexpr ui::model::ActionId enable{ 1 };
        static constexpr ui::model::ActionId disable{ 2 };
        static constexpr ui::model::ActionId clearFaults{ 3 };
        static constexpr ui::model::ActionId rearm{ 4 };
    };

    struct SingleStepItem
    {
        Control control;
        std::string_view label;
    };

    inline constexpr std::array<SingleStepItem, 4> singleStepItems{ {
        { Control::checkContracts, "Check &Contracts" },
        { Control::positionMode, "&Position Mode" },
        { Control::start, "&Start" },
        { Control::stop, "S&top" },
    } };

    struct MoveAction
    {
        static constexpr ui::model::ActionId move{ 0 };
        static constexpr ui::model::ActionId abort{ 1 };
    };

    class SimulatorPresenter
    {
    public:
        static constexpr std::size_t staleFrames{ 30 };

        SimulatorPresenter(arm_host::ArmCommands& arm, ui::shell::ShellView& shell, ui::shell::FormView& operateView, ui::shell::FormView& moveView,
            const arm::JointLimits& limits, infra::TimePoint start);
        SimulatorPresenter(const SimulatorPresenter& other) = delete;
        SimulatorPresenter& operator=(const SimulatorPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events);
        void EmergencyStop();
        void SwitchTarget(const arm_host::ArmTarget& target);
        void LoadProfile(const arm::ArmProfile& profile);
        void ConfirmRearm();
        bool InjectFault(std::size_t joint);
        bool SilenceJoint(std::size_t joint);
        bool SelectJoint(int joint);
        bool SingleStep(Control control);

        const ControlState& State(Control control) const;
        const StatusStrip& Status() const;
        const std::array<JointRow, arm::jointCount>& Rows() const;
        const ToolReadout& Tool() const;
        const ArmSceneModel& Scene() const;
        const EventLog& Log() const;
        const std::string& OperateHint() const;
        const std::string& MoveHint() const;
        ui::model::FormModel& MoveForm();
        void SetRearmPrompt(const ui::Callback<void(std::string_view prompt)>& prompt);

    private:
        struct Forms
        {
            explicit Forms(const arm::JointLimits& limits);
            Forms(const Forms& other) = delete;
            Forms& operator=(const Forms& other) = delete;

            std::array<ui::model::FieldSpec, arm::jointCount> moveFields;
            ui::model::FormSpec operateSpec;
            ui::model::FormSpec moveSpec;
            std::array<ui::model::FieldValue, arm::jointCount> moveValues{};
            ui::model::FormModel operateModel;
            ui::model::FormModel moveModel;
        };

        void OnOperateAction(ui::model::ActionId action);
        void OnMoveAction(ui::model::ActionId action);
        void Post(const arm_host::ArmCommand& command);
        void Handle(const arm_host::ArmEvent& event);
        void UpdateControls();
        void UpdateStatus();
        void TrackSequence();
        void FollowLimits();
        std::string TargetProblem() const;
        std::string Hint(std::span<const std::pair<Control, std::string_view>> controls) const;

        arm_host::ArmCommands& arm;
        ui::shell::ShellView& shell;
        ui::shell::FormView& operateView;
        ui::shell::FormView& moveView;
        arm::JointLimits limits;
        Forms forms{ limits };
        arm_host::ArmSnapshot snapshot;
        std::array<ControlState, controlCount> states;
        std::array<JointRow, arm::jointCount> rows;
        ToolReadout tool;
        StatusStrip status;
        ArmSceneModel scene;
        EventLog log;
        std::string tripSource;
        std::string operateHint;
        std::string moveHint;
        std::map<uint32_t, std::string> pending;
        ui::Callback<void(std::string_view prompt)> rearmPrompt;
        uint64_t lastSequence{ 0 };
        std::size_t unchangedFrames{ 0 };
    };
}

#endif
