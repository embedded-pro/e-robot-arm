#ifndef TOOLS_SIMULATOR_VIEW_MODELS_SIMULATOR_PRESENTER_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_SIMULATOR_PRESENTER_HPP

#include "tools/arm_host/ArmCommands.hpp"
#include "tools/simulator/view_models/ArmSceneModel.hpp"
#include "tools/simulator/view_models/ControlRules.hpp"
#include "tools/simulator/view_models/EventLog.hpp"
#include "tools/simulator/view_models/JointRows.hpp"
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
        ui::theme::StatusLevel connectionLevel{ ui::theme::StatusLevel::Neutral };
        std::string contracts;
        std::string trip;
        ui::theme::StatusLevel tripLevel{ ui::theme::StatusLevel::Neutral };
        bool realArm{ false };
    };

    class SimulatorPresenter
    {
    public:
        static constexpr ui::model::ActionId checkContracts{ 0 };
        static constexpr ui::model::ActionId positionMode{ 1 };
        static constexpr ui::model::ActionId start{ 2 };
        static constexpr ui::model::ActionId stop{ 3 };
        static constexpr ui::model::ActionId clearFaults{ 4 };
        static constexpr ui::model::ActionId rearm{ 5 };
        static constexpr ui::model::ActionId move{ 0 };
        static constexpr ui::model::ActionId abort{ 1 };

        SimulatorPresenter(arm_host::ArmCommands& arm, ui::shell::ShellView& shell, ui::shell::FormView& operateView, ui::shell::FormView& moveView,
            const arm::JointLimits& limits, infra::TimePoint start);
        SimulatorPresenter(const SimulatorPresenter& other) = delete;
        SimulatorPresenter& operator=(const SimulatorPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events);
        void EmergencyStop();
        void ConfirmRearm();
        bool InjectFault(std::size_t joint);
        bool SilenceJoint(std::size_t joint);
        bool SelectJoint(int joint);

        const ControlState& State(Control control) const;
        const StatusStrip& Status() const;
        const std::array<JointRow, arm::jointCount>& Rows() const;
        const ArmSceneModel& Scene() const;
        const EventLog& Log() const;
        const std::string& OperateHint() const;
        const std::string& MoveHint() const;
        ui::model::FormModel& MoveForm();

        ui::Callback<void(std::string_view prompt)> onRearmRequested;

    private:
        void OnOperateAction(ui::model::ActionId action);
        void OnMoveAction(ui::model::ActionId action);
        void Post(const arm_host::ArmCommand& command);
        void Handle(const arm_host::ArmEvent& event);
        void UpdateControls();
        void UpdateStatus();
        std::string TargetProblem() const;
        std::string Hint(std::span<const std::pair<Control, std::string_view>> controls) const;

        arm_host::ArmCommands& arm;
        ui::shell::ShellView& shell;
        ui::shell::FormView& operateView;
        ui::shell::FormView& moveView;
        arm::JointLimits limits;
        std::array<ui::model::FieldSpec, arm::jointCount> moveFields;
        ui::model::FormSpec operateSpec;
        ui::model::FormSpec moveSpec;
        std::array<ui::model::FieldValue, arm::jointCount> moveValues{};
        ui::model::FormModel operateModel;
        ui::model::FormModel moveModel;
        arm_host::ArmSnapshot snapshot;
        std::array<ControlState, 10> states;
        std::array<JointRow, arm::jointCount> rows;
        StatusStrip status;
        ArmSceneModel scene;
        EventLog log;
        std::string tripSource;
        std::string operateHint;
        std::string moveHint;
        std::map<uint32_t, std::string> pending;
    };
}

#endif
