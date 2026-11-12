#ifndef TOOLS_ARM_HOST_ARM_SESSION_HPP
#define TOOLS_ARM_HOST_ARM_SESSION_HPP

#include "core/arm/Arm.hpp"
#include "e-foc-protocol/client/FocMotorCanClient.hpp"
#include "infra/timer/Timer.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulated_arm/SimulatedArm.hpp"
#include <deque>

namespace arm_host
{
    class ArmSession
    {
    public:
        static constexpr infra::Duration retryInterval{ std::chrono::milliseconds{ 1 } };
        static constexpr uint8_t maxAttempts{ 20 };

        ArmSession(hal::Can& can, const arm::ArmConfig& config, const ArmTarget& target, ArmSessionSink& sink, simulation::SimulatedArm* simulation);
        ArmSession(const ArmSession& other) = delete;
        ArmSession& operator=(const ArmSession& other) = delete;
        ~ArmSession();

        void Execute(uint32_t id, const ArmCommand& command);
        void StopAll();
        bool Moving() const;

    private:
        class Events
            : private arm::JointBusObserver
            , private arm::JointMotionObserver
        {
        public:
            Events(ArmSession& session, arm::Arm& arm);

        private:
            void OnContractsResolved(can::ContractCompatibility compatibility) override;
            void OnJointFault(std::size_t joint, can::FocFaultCode fault) override;
            void OnJointSilent(std::size_t joint) override;
            void OnMoveDone() override;
            void OnMoveAborted() override;

            ArmSession& session;
        };

        struct Runner;

        struct Pending
        {
            uint32_t id;
            ArmCommand command;
        };

        void Pump();
        bool QueueRefused(const ArmCommand& command) const;
        bool Run(const ArmCommand& command);
        bool Simulate(std::size_t joint, const infra::Function<void(simulation::SimulatedJoint&)>& action);
        std::string_view RefusalReason(const ArmCommand& command) const;
        void ReportTrip(std::size_t joint, can::FocFaultCode fault);
        void Emit(ArmEvent event);
        void Publish();

        can::FocMotorCanClient client;
        arm::Arm arm;
        ArmTarget target;
        ArmSessionSink& sink;
        simulation::SimulatedArm* simulation;
        Events events{ *this, arm };
        infra::TimerRepeating publishTimer;
        infra::TimerSingleShot retryTimer;
        std::deque<Pending> backlog;
        uint8_t attempts{ 0 };
        bool tripReported{ false };
    };
}

#endif
