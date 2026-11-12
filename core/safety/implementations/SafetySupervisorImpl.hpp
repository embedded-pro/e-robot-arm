#ifndef CORE_SAFETY_IMPLEMENTATIONS_SAFETY_SUPERVISOR_IMPL_HPP
#define CORE_SAFETY_IMPLEMENTATIONS_SAFETY_SUPERVISOR_IMPL_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/safety/interfaces/SafetySupervisor.hpp"

namespace arm
{
    class SafetySupervisorImpl
        : public SafetySupervisor
        , private JointBusObserver
    {
    public:
        SafetySupervisorImpl(JointBus& bus, const JointLimits& limits);

        bool QueryContracts() override;
        can::ContractCompatibility Contracts() const override;

        bool SelectMode(can::FocMotorMode mode) override;
        bool Start() override;
        bool Stop() override;
        bool ClearFault() override;
        bool EmergencyStop() override;
        bool SendPositions(const JointPositions& positions) override;
        bool RequestTelemetry() override;

        const JointReading& Reading(std::size_t joint) const override;

        bool Tripped() const override;
        bool Rearm() override;

    private:
        void OnContractsResolved(can::ContractCompatibility compatibility) override;
        void OnJointFault(std::size_t joint, can::FocFaultCode fault) override;
        void OnJointSilent(std::size_t joint) override;
        void OnCommandDelivered(JointCommand command) override;
        void OnCommandFailed(JointCommand command, CommandFailure failure) override;

        bool MayCommandMotion() const;
        bool AnyJointFaulted() const;
        void Trip();

        JointBus& bus;
        JointLimits limits;
        bool tripped{ false };
    };
}

#endif
