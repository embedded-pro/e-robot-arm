#pragma once

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
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        using Pid = controllers::PidIncrementalSynchronous<float>;

        JointAccelerations Corrections(const MeasuredJoints& measured, const JointReference& reference);

        const ArmModel& model;
        JointInertias rotors;
        float derivativeGain;
        std::array<Pid, jointCount> pids;
    };
}
