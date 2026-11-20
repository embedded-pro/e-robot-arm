#include "core/arm_controllers/implementations/SlotineLi.hpp"
#include <chrono>

namespace arm
{
    namespace
    {
        constexpr float adaptationFloor{ 0.001f };

        ArmModel::MassMatrix SlidingGain(const ArmModel& model, const JointInertias& rotors, float bandwidth)
        {
            auto inertia = model.Dynamics().ComputeMassMatrix(model.ModelVector(JointPositions{}));
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                inertia.at(joint, joint) += rotors[joint];
            return inertia * bandwidth;
        }

        math::SquareMatrix<float, SlotineLi::Parameters::rows> Adaptation(float gain, const SlotineLi::Parameters& nominal)
        {
            math::SquareMatrix<float, SlotineLi::Parameters::rows> adaptation{};
            for (std::size_t index = 0; index != SlotineLi::Parameters::rows; ++index)
                adaptation.at(index, index) = gain * (nominal.at(index, 0) * nominal.at(index, 0) + adaptationFloor * adaptationFloor);
            return adaptation;
        }
    }

    SlotineLi::SlotineLi(const ArmModel& model, const JointInertias& rotors, const SlotineLiTuning& tuning, infra::Duration period)
        : model(model)
        , rotors(rotors)
        , lambda(tuning.lambda)
        , regressor(model.Links(), model.Gravity())
        , law(regressor, ArmModel::MassMatrix::Identity() * tuning.lambda, SlidingGain(model, rotors, tuning.bandwidth), Adaptation(tuning.adaptationGain, Regressor::Parameters(model.Links())),
              Regressor::Parameters(model.Links()), std::chrono::duration<float>{ period }.count())
    {}

    const SlotineLi::Parameters& SlotineLi::Estimate() const
    {
        return law.ParameterEstimate();
    }

    void SlotineLi::ResetEstimate()
    {
        law.Reset(Regressor::Parameters(model.Links()));
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

        auto torques = law.ComputeTorque(model.ModelVector(measured.positions), speeds, model.ModelVector(reference.positions), targetSpeeds, targetAccelerations);
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            torques.at(joint, 0) += rotors[joint] * (targetAccelerations.at(joint, 0) - lambda * (speeds.at(joint, 0) - targetSpeeds.at(joint, 0)));
        return JointSetpoints{ reference.positions, model.ToJointTorques(torques) };
    }
}
