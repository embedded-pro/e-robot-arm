#pragma once

#include "core/joint_configurator/interfaces/JointConfigurator.hpp"
#include "core/lifecycle/interfaces/ArmLifecycle.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "core/safety/interfaces/SafetySupervisor.hpp"
#include "services/fsm/TableStateMachine.hpp"
#include <array>
#include <variant>

namespace arm::lifecycle
{
    struct Unpowered
    {
        static constexpr const char* name{ "Unpowered" };
    };

    struct Checking
    {
        static constexpr const char* name{ "Checking" };
    };

    struct Incompatible
    {
        static constexpr const char* name{ "Incompatible" };
    };

    struct WaitingForCalibration
    {
        static constexpr const char* name{ "WaitingForCalibration" };
    };

    struct Configuring
    {
        static constexpr const char* name{ "Configuring" };
    };

    struct Misconfigured
    {
        static constexpr const char* name{ "Misconfigured" };
    };

    struct Ready
    {
        static constexpr const char* name{ "Ready" };
    };

    struct Enabling
    {
        static constexpr const char* name{ "Enabling" };

        bool started{ false };
        uint32_t periodsStarting{ 0 };
    };

    struct Enabled
    {
        static constexpr const char* name{ "Enabled" };
    };

    struct Moving
    {
        static constexpr const char* name{ "Moving" };
    };

    struct Disabling
    {
        static constexpr const char* name{ "Disabling" };
    };

    struct Holding
    {
        static constexpr const char* name{ "Holding" };
    };

    struct Tripped
    {
        static constexpr const char* name{ "Tripped" };
    };

    using State = std::variant<Unpowered, Checking, Incompatible, WaitingForCalibration, Configuring, Misconfigured, Ready, Enabling, Enabled, Moving, Disabling, Holding, Tripped>;

    struct BringUp
    {
        static constexpr const char* name{ "BringUp" };
    };

    struct ContractsResolved
    {
        static constexpr const char* name{ "ContractsResolved" };
        can::ContractCompatibility compatibility;
    };

    struct Observed
    {
        static constexpr const char* name{ "Observed" };
        bool allCalibrated;
        bool allRunning;
        bool anyRunning;
    };

    struct Configured
    {
        static constexpr const char* name{ "Configured" };
    };

    struct Misconfigure
    {
        static constexpr const char* name{ "Misconfigure" };
        Misconfiguration misconfiguration;
    };

    struct Enable
    {
        static constexpr const char* name{ "Enable" };
    };

    struct Disable
    {
        static constexpr const char* name{ "Disable" };
    };

    struct MoveStarted
    {
        static constexpr const char* name{ "MoveStarted" };
    };

    struct MoveFinished
    {
        static constexpr const char* name{ "MoveFinished" };
    };

    struct Delivered
    {
        static constexpr const char* name{ "Delivered" };
        JointCommand command;
    };

    struct Failed
    {
        static constexpr const char* name{ "Failed" };
        JointCommand command;
    };

    struct Trip
    {
        static constexpr const char* name{ "Trip" };
    };

    struct ProtectiveStop
    {
        static constexpr const char* name{ "ProtectiveStop" };
    };

    struct Rearmed
    {
        static constexpr const char* name{ "Rearmed" };
    };

    using Event = std::variant<BringUp, ContractsResolved, Observed, Configured, Misconfigure, Enable, Disable, MoveStarted, MoveFinished, Delivered, Failed, Trip, ProtectiveStop, Rearmed>;
}

namespace arm
{
    class ArmLifecycleImpl
        : public ArmLifecycle
        , private JointBusObserver
        , private JointMotionObserver
        , private JointConfiguratorObserver
    {
    public:
        using Machine = services::TableStateMachine<lifecycle::State, lifecycle::Event, ArmLifecycleImpl>;
        using Initial = lifecycle::Unpowered;

        static constexpr std::size_t rowCount{ 42 };
        static constexpr uint32_t restartPeriods{ 20 };
        static constexpr uint32_t giveUpPeriods{ 200 };
        static constexpr std::array<Machine::Transition, rowCount> Rows();
        static constexpr Machine::Rules Rules();

        ArmLifecycleImpl(SafetySupervisor& safety, JointMotion& motion, JointConfigurator& configurator, const JointControllers& controllers,
            const JointMotors& motors = {}, std::optional<float> watchdogSeconds = std::nullopt);

        bool BringUp() override;
        bool Enable() override;
        bool Disable() override;
        bool Rearm() override;
        bool MoveTo(const JointPositions& target) override;
        bool StepTo(const JointPositions& target) override;
        bool MoveThrough(const JointWaypoints& waypoints) override;
        bool MoveLine(const ToolPose& target) override;
        bool Jog(const JointJog& jog) override;
        bool Jog(const ToolJog& jog) override;
        ArmState State() const override;
        std::optional<Misconfiguration> MisconfiguredJoint() const override;
        std::optional<std::size_t> UnstartedJoint() const override;

        void Step();

    private:
        void OnContractsResolved(can::ContractCompatibility compatibility) override;
        void OnJointFault(std::size_t joint, can::FocFaultCode fault) override;
        void OnJointSilent(std::size_t joint) override;
        void OnSupervisionTrip(std::size_t joint, SupervisionTrip trip, float value) override;
        void OnProtectiveStop(std::size_t joint, SupervisionTrip trip, float value) override;
        void OnHardwareStop(bool open) override;
        void OnCommandDelivered(JointCommand command) override;
        void OnCommandFailed(JointCommand command, CommandFailure failure) override;
        void OnMoveDone() override;
        void OnMoveAborted() override;
        void OnApplied(std::size_t joint) override;
        void OnRead(std::size_t joint, const ControllerReading& reading) override;
        void OnMotorApplied(std::size_t joint) override;
        void OnMotorRead(std::size_t joint, const MotorSettings& motor) override;
        void OnWatchdogApplied(std::size_t joint) override;
        void OnWatchdogRead(std::size_t joint, float timeoutSeconds) override;
        void OnConfigurationFailed(std::size_t joint, ConfigurationFailure failure) override;

        class StateReporter
            : public services::StateMachineObserver<lifecycle::State, lifecycle::Event>
        {
        public:
            StateReporter(ArmLifecycleImpl& lifecycle, Machine& machine);

            void StateChanged(StateId from, const lifecycle::Event& event, StateId to) override;

        private:
            ArmLifecycleImpl& lifecycle;
        };

        bool Accepted(const lifecycle::Event& event);
        lifecycle::Observed Observe() const;
        void QueryContracts();
        void SelectMode();
        void StartJoints();
        void KeepStarting(lifecycle::Enabling& enabling);
        void GiveUpStarting();
        void StopJoints();
        bool HasConfiguration() const;
        bool Configures(std::size_t configurationStep) const;
        bool StartStep(std::size_t configurationStep);
        static std::size_t JointOf(std::size_t configurationStep);
        static ConfigurationPart PartOf(std::size_t configurationStep);
        bool WorkingOn(std::size_t joint, ConfigurationPart part) const;
        void Verified();
        void StartConfiguring();
        void ConfigureNext();
        void Misconfigure(MisconfigurationReason reason);

        SafetySupervisor& safety;
        JointMotion& motion;
        JointConfigurator& configurator;
        JointControllers controllers;
        JointMotors motors;
        std::optional<float> watchdogSeconds;
        std::size_t nextStep{ 0 };
        std::optional<std::size_t> configuringStep;
        std::optional<Misconfiguration> misconfiguration;
        std::optional<std::size_t> unstartedJoint;
        Machine::WithStorage<4> fsm;
        StateReporter reporter{ *this, fsm };
    };
}
