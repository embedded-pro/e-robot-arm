#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_COMPUTED_TORQUE_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_COMPUTED_TORQUE_HPP

#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"
#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/ArmModel.hpp"
#include "robotics/controllers/manipulator/ComputedTorqueControl.hpp"
#include "robotics/dynamics/InverseDynamicsModel.hpp"

namespace arm
{
    class ArmatureDynamics
        : public dynamics::InverseDynamicsModel<float, jointCount>
    {
    public:
        ArmatureDynamics(const dynamics::InverseDynamicsModel<float, jointCount>& arm, const JointInertias& rotors);

        StateVector ComputeInverseDynamics(const StateVector& q, const StateVector& qDot, const StateVector& qDDot) const override;

    private:
        const dynamics::InverseDynamicsModel<float, jointCount>& arm;
        JointInertias rotors;
    };

    class ComputedTorque
        : public ArmController
    {
    public:
        ComputedTorque(const ArmModel& model, const JointInertias& rotors, const BandwidthTuning& tuning);

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        using Law = controllers::ComputedTorqueControl<float, jointCount>;

        const ArmModel& model;
        ArmatureDynamics dynamics;
        Law law;
    };
}

#endif
