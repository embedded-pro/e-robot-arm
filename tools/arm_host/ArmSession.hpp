#ifndef TOOLS_ARM_HOST_ARM_SESSION_HPP
#define TOOLS_ARM_HOST_ARM_SESSION_HPP

#include "core/arm/Arm.hpp"
#include "core/arm_profile/ArmProfile.hpp"
#include "e-foc-protocol/client/FocMotorCanClient.hpp"
#include "infra/timer/Timer.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulated_arm/SimulatedArm.hpp"
#include <optional>

namespace arm_host
{
    class ArmSession
    {
    public:
        ArmSession(hal::Can& can, const arm::ArmProfile& profile, const ArmTarget& target, ArmSessionSink& sink, simulation::SimulatedArm* simulation);
        ArmSession(const ArmSession& other) = delete;
        ArmSession& operator=(const ArmSession& other) = delete;
        ~ArmSession();

        void Execute(uint32_t id, const ArmCommand& command);
        void StopAll();
        bool Moving() const;
        bool Running() const;

    private:
        class Events
            : private arm::JointBusObserver
            , private arm::JointMotionObserver
            , private arm::ArmLifecycleObserver
            , private arm::JointConfiguratorObserver
        {
        public:
            Events(ArmSession& session, arm::Arm& arm);

        private:
            void OnContractsResolved(can::ContractCompatibility compatibility) override;
            void OnJointFault(std::size_t joint, can::FocFaultCode fault) override;
            void OnJointSilent(std::size_t joint) override;
            void OnCommandDelivered(arm::JointCommand command) override;
            void OnCommandFailed(arm::JointCommand command, arm::CommandFailure failure) override;
            void OnMoveDone() override;
            void OnMoveAborted() override;
            void OnStateChanged(arm::ArmState state) override;
            void OnApplied(std::size_t joint) override;
            void OnRead(std::size_t joint, const arm::ControllerReading& reading) override;
            void OnMotorApplied(std::size_t joint) override;
            void OnMotorRead(std::size_t joint, const arm::MotorSettings& motor) override;
            void OnConfigurationFailed(std::size_t joint, arm::ConfigurationFailure failure) override;

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
        bool Simulate(std::size_t joint, const infra::Function<void(simulation::SimulatedJoint&)>& action);
        std::string_view RefusalReason(const ArmCommand& command) const;
        void ReportTrip(std::size_t joint, can::FocFaultCode fault);
        void Emit(ArmEvent event);
        void Publish();

        arm::ArmProfile profile;
        can::FocMotorCanClient client;
        arm::Arm arm;
        arm::ArmModel model;
        ArmTarget target;
        ArmSessionSink& sink;
        simulation::SimulatedArm* simulation;
        Events events{ *this, arm };
        infra::TimerRepeating publishTimer;
        std::optional<Pending> issuing;
        std::optional<Pending> held;
        bool tripReported{ false };
    };
}

#endif
