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
        bool SelectAlgorithm(std::size_t joint, can::FocControlLoop loop, uint8_t algorithm) override;
        bool QueryAlgorithms(std::size_t joint) override;
        bool SetParameter(std::size_t joint, can::FocParameter parameter, float value) override;
        bool QueryParameter(std::size_t joint, can::FocParameter parameter) override;

        const JointReading& Reading(std::size_t joint) const override;
        uint32_t Period() const override;

        bool Tripped() const override;
        bool Rearm() override;

    private:
        void OnContractsResolved(can::ContractCompatibility compatibility) override;
        void OnJointFault(std::size_t joint, can::FocFaultCode fault) override;
        void OnJointSilent(std::size_t joint) override;
        void OnCommandDelivered(JointCommand command) override;
        void OnCommandFailed(JointCommand command, CommandFailure failure) override;
        void OnAlgorithms(std::size_t joint, const can::FocAlgorithms& algorithms) override;
        void OnParameter(std::size_t joint, can::FocParameter parameter, can::FocParameterValue value) override;

        bool MayCommandMotion() const;
        bool AnyJointFaulted() const;
        void Trip();

        JointBus& bus;
        JointLimits limits;
        bool tripped{ false };
    };
}

#endif
