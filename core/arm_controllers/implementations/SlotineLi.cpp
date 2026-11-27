#include "core/arm_controllers/implementations/SlotineLi.hpp"
#include <chrono>

namespace arm
{
    namespace
    {
        constexpr float adaptationFloor{ 0.001f };

        ArmModel::MassMatrix SlidingGain(const DrivenArm& drivenArm, float bandwidth)
        {
            return drivenArm.MassMatrix(drivenArm.Model().ModelVector(JointPositions{})) * bandwidth;
        }

        math::SquareMatrix<float, SlotineLi::Parameters::rows> Adaptation(float gain, const SlotineLi::Parameters& nominal)
        {
            math::SquareMatrix<float, SlotineLi::Parameters::rows> adaptation{};
            for (std::size_t index = 0; index != SlotineLi::Parameters::rows; ++index)
                adaptation.at(index, index) = gain * (nominal.at(index, 0) * nominal.at(index, 0) + adaptationFloor * adaptationFloor);
            return adaptation;
        }
    }

    SlotineLi::SlotineLi(const DrivenArm& drivenArm, const SlotineLiTuning& tuning, infra::Duration period)
        : drivenArm(drivenArm)
        , model(drivenArm.Model())
        , lambda(tuning.lambda)
        , regressor(model.Links(), model.Gravity())
        , periodSeconds(std::chrono::duration<float>{ period }.count())
    {
        Emplace(tuning, Regressor::Parameters(model.Links()));
    }

    void SlotineLi::SetTuning(const SlotineLiTuning& tuning)
    {
        const auto learnt = law->ParameterEstimate();
        Emplace(tuning, learnt);
    }

    void SlotineLi::Emplace(const SlotineLiTuning& tuning, const Parameters& estimate)
    {
        lambda = tuning.lambda;
        law.emplace(regressor, ArmModel::MassMatrix::Identity() * tuning.lambda, SlidingGain(drivenArm, tuning.bandwidth),
            Adaptation(tuning.adaptationGain, Regressor::Parameters(model.Links())), estimate, periodSeconds);
    }

    const SlotineLi::Parameters& SlotineLi::Estimate() const
    {
        return law->ParameterEstimate();
    }

    void SlotineLi::ResetEstimate()
    {
        law->Reset(Regressor::Parameters(model.Links()));
    }

    bool SlotineLi::BumplessTransfer() const
    {
        return true;
    }

    void SlotineLi::Reset(const MeasuredJoints&, const JointReference&)
    {}

    JointSetpoints SlotineLi::Step(const MeasuredJoints& measured, const JointReference& reference)
    {
        const auto speeds = model.ModelSpeeds(measured.speeds);
        const auto targetSpeeds = model.ModelSpeeds(reference.speeds);
        const auto targetAccelerations = model.ModelAccelerations(reference.accelerations);

        auto torques = law->ComputeTorque(model.ModelVector(measured.positions), speeds, model.ModelVector(reference.positions), targetSpeeds, targetAccelerations);
        torques += math::Diagonal(ToVector(drivenArm.Rotors())) * JointVector{ targetAccelerations - (speeds - targetSpeeds) * lambda };
        return JointSetpoints{ reference.positions, model.ToJointTorques(torques) };
    }
}
