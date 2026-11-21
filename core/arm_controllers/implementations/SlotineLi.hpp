#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_SLOTINE_LI_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_SLOTINE_LI_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/DrivenArm.hpp"
#include "infra/timer/Timer.hpp"
#include "robotics/controllers/manipulator/SlotineLiAdaptiveControl.hpp"
#include "robotics/dynamics/ChainInertialRegressor.hpp"

namespace arm
{
    struct SlotineLiTuning
    {
        float lambda{ 10.0f };
        float bandwidth{ 20.0f };
        float adaptationGain{ 10.0f };

        bool operator==(const SlotineLiTuning& other) const = default;
    };

    class SlotineLi
        : public ArmController
    {
    public:
        using Regressor = dynamics::ChainInertialRegressor<float, jointCount>;
        using Parameters = Regressor::ParameterVector;

        SlotineLi(const DrivenArm& drivenArm, const SlotineLiTuning& tuning, infra::Duration period);

        const Parameters& Estimate() const;
        void ResetEstimate();

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        using Law = controllers::SlotineLiAdaptiveControl<float, jointCount, Regressor::ParametersPerLink * jointCount>;

        const DrivenArm& drivenArm;
        const ArmModel& model;
        float lambda;
        Regressor regressor;
        Law law;
    };
}

#endif
