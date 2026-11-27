#pragma once

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/DrivenArm.hpp"
#include "infra/timer/Timer.hpp"
#include "robotics/controllers/manipulator/SlotineLiAdaptiveControl.hpp"
#include "robotics/dynamics/ChainInertialRegressor.hpp"
#include <optional>

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

        void SetTuning(const SlotineLiTuning& tuning);
        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        using Law = controllers::SlotineLiAdaptiveControl<float, jointCount, Regressor::ParametersPerLink * jointCount>;

        void Emplace(const SlotineLiTuning& tuning, const Parameters& estimate);

        const DrivenArm& drivenArm;
        const ArmModel& model;
        float lambda;
        Regressor regressor;
        float periodSeconds;
        std::optional<Law> law;
    };
}
