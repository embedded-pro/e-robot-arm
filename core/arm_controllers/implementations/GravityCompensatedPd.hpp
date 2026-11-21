#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_GRAVITY_COMPENSATED_PD_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_GRAVITY_COMPENSATED_PD_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/DrivenArm.hpp"
#include "robotics/controllers/manipulator/PdGravityCompensation.hpp"
#include <array>

namespace arm
{
    using PdGainMatrix = controllers::PdGravityCompensation<float, jointCount>::GainMatrix;

    struct PdGainMatrices
    {
        PdGainMatrix kp;
        PdGainMatrix kd;
    };

    struct BandwidthTuning
    {
        float bandwidth{ 30.0f };
        float dampingRatio{ 0.7f };
    };

    PdGainMatrices PlacedGains(const DrivenArm& drivenArm, const BandwidthTuning& tuning, const JointPositions& positions);

    class GravityCompensatedPd
        : public ArmController
    {
    public:
        GravityCompensatedPd(const DrivenArm& drivenArm, const BandwidthTuning& tuning);

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        using Law = controllers::PdGravityCompensation<float, jointCount>;

        const DrivenArm& drivenArm;
        const ArmModel& model;
        BandwidthTuning tuning;
    };
}

#endif
