#ifndef TOOLS_ARM_HOST_ARM_SESSION_HPP
#define TOOLS_ARM_HOST_ARM_SESSION_HPP

#include "core/arm/Arm.hpp"
#include "core/arm_profile/ArmProfile.hpp"
#include "e-foc-protocol/client/FocMotorCanClient.hpp"
#include "infra/timer/Timer.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/arm_host/VirtualMotors.hpp"
#include "tools/simulated_arm/SimulatedArm.hpp"
#include <optional>
#include <string>

namespace arm_host
{
    class ArmSession
    {
    public:
        ArmSession(hal::Can& can, const arm::ArmProfile& profile, const ArmTarget& target, ArmSessionSink& sink, simulation::SimulatedArm* simulation,
            VirtualMotors* virtualMotors = nullptr, bool commissioning = false);
        ArmSession(const ArmSession& other) = delete;
        ArmSession& operator=(const ArmSession& other) = delete;
        ~ArmSession();

        void Execute(uint32_t id, const ArmCommand& command);
        void StopAll();
        bool Moving() const;
        bool Running() const;
        bool DrivesJoints() const;

    private:
        class Events
            : private arm::JointBusObserver
            , private arm::JointMotionObserver
            , private arm::ArmLifecycleObserver
            , private arm::JointConfiguratorObserver
            , private arm::ArmControllersObserver
        {
        public:
            Events(ArmSession& session, arm::Arm& arm);

        private:
            void OnContractsResolved(can::ContractCompatibility compatibility) override;
            void OnJointFault(std::size_t joint, can::FocFaultCode fault) override;
            void OnJointSilent(std::size_t joint) override;
            void OnSupervisionTrip(std::size_t joint, arm::SupervisionTrip trip, float value) override;
            void OnProtectiveStop(std::size_t joint, arm::SupervisionTrip trip, float value) override;
            void OnHardwareStop(bool open) override;
            void OnCommandDelivered(arm::JointCommand command) override;
            void OnCommandFailed(arm::JointCommand command, arm::CommandFailure failure) override;
            void OnMoveDone() override;
            void OnMoveAborted() override;
            void OnStateChanged(arm::ArmState state) override;
            void OnApplied(std::size_t joint) override;
            void OnRead(std::size_t joint, const arm::ControllerReading& reading) override;
            void OnMotorApplied(std::size_t joint) override;
            void OnMotorRead(std::size_t joint, const arm::MotorSettings& motor) override;
            void OnWatchdogApplied(std::size_t joint) override;
            void OnWatchdogRead(std::size_t joint, float timeoutSeconds) override;
            void OnConfigurationFailed(std::size_t joint, arm::ConfigurationFailure failure) override;
            void OnActiveControllerChanged(std::optional<arm::ArmControllerId> active) override;

            ArmSession& session;
        };

        struct Runner;

        struct Pending
        {
            Pending(uint32_t id, const ArmCommand& command)
                : id(id)
                , command(command)
            {}

            uint32_t id;
            ArmCommand command;
        };

        void Deliver(uint32_t id, const ArmCommand& command);
        void Delivered(arm::JointCommand command);
        void Failed(arm::JointCommand command, arm::CommandFailure failure);
        bool Run(const ArmCommand& command);
        bool Refuse(std::string_view reason);
        bool InjectFaultInto(const InjectFault& inject);
        bool SetSilent(std::size_t joint, bool silent);
        bool Collide(const CollideJoint& collide);
        bool PowerOff(std::size_t joint);
        bool RestartVirtualMotor(std::size_t joint);
        void PublishDisturbances(ArmSnapshot& snapshot) const;
        std::string_view RefusalReason(const ArmCommand& command) const;
        std::string_view ArmControllerRefusal(arm::ArmControllerId id) const;
        std::optional<std::string_view> MotionPermissionRefusal() const;
        std::string_view MoveRefusal(const ArmCommand& command) const;
        void ReportTrip(std::size_t joint, can::FocFaultCode fault, std::string reason = {});
        void Emit(ArmEvent event);
        void Publish();

        arm::ArmProfile profile;
        bool commissioning;
        can::FocMotorCanClient client;
        arm::Arm arm;
        arm::ArmModel model;
        ArmTarget target;
        ArmSessionSink& sink;
        simulation::SimulatedArm* simulation;
        VirtualMotors* virtualMotors;
        std::string refusal;
        mutable std::string workspaceRefusal;
        Events events{ *this, arm };
        infra::TimerRepeating publishTimer;
        std::optional<Pending> issuing;
        std::optional<Pending> held;
        bool tripReported{ false };
        std::optional<StopReport> stop;
    };
}

#endif
