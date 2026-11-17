#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_GRAVITY_PID_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_GRAVITY_PID_HPP

#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"
#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/ArmModel.hpp"
#include "infra/timer/Timer.hpp"
#include "numerical/controllers/implementations/PidIncremental.hpp"
#include <array>

namespace arm
{
    class GravityPid
        : public ArmController
    {
    public:
        static constexpr float correctionLimit{ 200.0f };

        GravityPid(const ArmModel& model, const JointInertias& rotors, const BandwidthTuning& tuning, infra::Duration period);

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        using Pid = controllers::PidIncrementalSynchronous<float>;

        JointAccelerations Corrections(const MeasuredJoints& measured, const JointReference& reference);

        const ArmModel& model;
        ArmModel::MassMatrix inertia;
        float derivativeGain;
        std::array<Pid, jointCount> pids;
        bool primed{ false };
    };
}

#endif
