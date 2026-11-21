#pragma once

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/safety/interfaces/SafetySupervisor.hpp"

namespace arm
{
    class SafetySupervisorImpl
        : public SafetySupervisor
        , private JointBusObserver
        , private HardwareStopObserver
    {
    public:
        SafetySupervisorImpl(JointBus& bus, const JointLimits& limits, const TrackingLimits& tracking, const CollisionLimits& collision, infra::Duration period,
            const MotionSupervision& motion = {}, const SupervisionInputs& inputs = {});

        bool QueryContracts() override;
        can::ContractCompatibility Contracts() const override;

        bool SelectMode(can::FocMotorMode mode) override;
        bool Start() override;
        bool Stop() override;
        bool ClearFault() override;
        bool Align() override;
        bool EmergencyStop() override;
        bool SendPositions(const JointPositions& positions) override;
        bool SendTorques(const JointTorques& torques) override;
        bool RequestTelemetry() override;
        bool SelectAlgorithm(std::size_t joint, can::FocControlLoop loop, uint8_t algorithm) override;
        bool QueryAlgorithms(std::size_t joint) override;
        bool SetParameter(std::size_t joint, can::FocParameter parameter, float value) override;
        bool QueryParameter(std::size_t joint, can::FocParameter parameter) override;

        const JointReading& Reading(std::size_t joint) const override;
        uint32_t Period() const override;

        bool Tripped() const override;
        bool Holding() const override;
        bool Rearm() override;
        void Supervise(const std::optional<JointReference>& reference, const std::optional<JointTorques>& externalTorques) override;
        const JointFlags& TorqueClamped() const override;
        std::optional<WorkspaceViolation> LastWorkspaceViolation() const override;
        bool HardwareStopOpen() const override;

    private:
        void OnContractsResolved(can::ContractCompatibility compatibility) override;
        void OnJointFault(std::size_t joint, can::FocFaultCode fault) override;
        void OnJointSilent(std::size_t joint) override;
        void OnCommandDelivered(JointCommand command) override;
        void OnCommandFailed(JointCommand command, CommandFailure failure) override;
        void OnAlgorithms(std::size_t joint, const can::FocAlgorithms& algorithms) override;
        void OnParameter(std::size_t joint, can::FocParameter parameter, can::FocParameterValue value) override;
        void OnHardwareStopChanged(bool open) override;

        bool MayCommandMotion() const;
        bool MayStream() const;
        bool AnyJointFaulted() const;
        void Trip();
        bool SuperviseJoint(std::size_t joint, foc::Radians reference);
        bool SuperviseCollision(const JointTorques& externalTorques);
        bool SuperviseReference(const JointReference& reference);
        bool SuperviseSpeed(std::size_t joint);
        bool SuperviseWorkspace(const JointPositions& reference);
        std::optional<JointPositions> FreshMeasuredPose() const;
        void SuperviseHold(const JointPositions& held);
        std::optional<float> TrackingErrorHeld(std::size_t joint, foc::Radians reference);
        void Protect(std::size_t joint, SupervisionTrip cause, float value);

        JointBus& bus;
        JointLimits limits;
        TrackingLimits tracking;
        CollisionLimits collision;
        MotionSupervision motion;
        float periodSeconds;
        std::optional<JointPositions> previousReference;
        JointFlags torqueClamped{};
        const PoseGuard* guard;
        HardwareStop* hardwareStop;
        std::optional<WorkspaceViolation> lastWorkspaceViolation;
        std::array<uint32_t, jointCount> stepsToTrip{};
        std::array<uint32_t, jointCount> stepsOutside{};
        bool tripped{ false };
        bool holding{ false };
    };
}
