#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_GRAVITY_COMPENSATED_PD_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_GRAVITY_COMPENSATED_PD_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/ArmModel.hpp"
#include "robotics/controllers/manipulator/PdGravityCompensation.hpp"
#include <array>

namespace arm
{
    struct PdGain
    {
        float kp{ 0.0f };
        float kd{ 0.0f };
    };

    using JointPdGains = std::array<PdGain, jointCount>;
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

    PdGainMatrices DiagonalGains(const JointPdGains& gains);
    PdGainMatrices PlacedGains(const ArmModel& model, const JointInertias& rotors, const BandwidthTuning& tuning);

    class GravityCompensatedPd
        : public ArmController
    {
    public:
        GravityCompensatedPd(const ArmModel& model, const PdGainMatrices& gains);

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        using Law = controllers::PdGravityCompensation<float, jointCount>;

        const ArmModel& model;
        Law law;
    };
}

#endif
