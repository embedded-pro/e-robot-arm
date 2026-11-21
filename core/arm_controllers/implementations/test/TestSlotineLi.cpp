#include "core/arm_controllers/implementations/SlotineLi.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <cmath>

namespace
{
    using namespace std::chrono_literals;

    constexpr float tolerance{ 1.0e-3f };
    constexpr infra::Duration period{ 1ms };

    arm::JointLimits WideLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        return limits;
    }

    arm::ArmMass WithPayload()
    {
        auto mass = arm::ReferenceMass();
        mass.payload = { 0.4f, { 0.0f, 0.0f, 0.05f }, { 0.0002f, 0.0002f, 0.0001f, 0.0f, 0.0f, 0.0f } };
        return mass;
    }

    bool Same(const arm::SlotineLi::Parameters& left, const arm::SlotineLi::Parameters& right)
    {
        for (std::size_t index = 0; index != arm::SlotineLi::Parameters::rows; ++index)
            if (left.at(index, 0) != right.at(index, 0))
                return false;
        return true;
    }

    class SlotineLiTest
        : public testing::Test
    {
    public:
        SlotineLiTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                measured.positions[joint] = foc::Radians{ 0.1f + 0.1f * static_cast<float>(joint) };
        }

        float TrackingError(float adaptationGain) const
        {
            const arm::ArmModel plant{ arm::ReferenceGeometry(), WithPayload(), WideLimits() };
            arm::SlotineLi adaptive{ drivenArm, arm::SlotineLiTuning{ 10.0f, 20.0f, adaptationGain }, period };

            auto angles = model.ModelVector(Reference(0.0f).positions);
            arm::ArmModel::JointVector speeds{};
            float squared{ 0.0f };
            std::size_t samples{ 0 };
            constexpr float dt{ 0.001f };
            for (std::size_t tick = 0; tick != 8000; ++tick)
            {
                const auto time = dt * static_cast<float>(tick);
                const auto reference = Reference(time);
                arm::MeasuredJoints joints{ plant.ToJointPositions(angles), plant.ToJointSpeeds(speeds) };
                const auto torques = plant.ModelTorques(adaptive.Step(joints, reference).torques);

                auto inertia = plant.Dynamics().ComputeMassMatrix(angles);
                for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                    inertia.at(joint, joint) += rotors[joint];
                const arm::ArmModel::JointVector free{ torques - plant.Dynamics().ComputeCoriolisTerms(angles, speeds) - plant.Dynamics().ComputeGravityTerms(angles) };
                speeds = speeds + solvers::SolveSystem(inertia, free) * dt;
                angles = angles + speeds * dt;

                if (tick >= 6000)
                {
                    const auto target = model.ModelVector(reference.positions);
                    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                        squared += (target.at(joint, 0) - angles.at(joint, 0)) * (target.at(joint, 0) - angles.at(joint, 0));
                    ++samples;
                }
            }
            return std::sqrt(squared / static_cast<float>(samples));
        }

        static arm::JointReference Reference(float time)
        {
            arm::JointReference reference{};
            const std::array<float, arm::jointCount> amplitude{ 0.3f, 0.4f, 0.4f, 0.3f, 0.3f, 0.3f };
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                const auto rate = 2.0f + 0.5f * static_cast<float>(joint);
                reference.positions[joint] = foc::Radians{ 0.3f + amplitude[joint] * std::sin(rate * time) };
                reference.speeds[joint] = foc::RadiansPerSecond{ amplitude[joint] * rate * std::cos(rate * time) };
                reference.accelerations[joint] = foc::RadiansPerSecondSquared{ -amplitude[joint] * rate * rate * std::sin(rate * time) };
            }
            return reference;
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        const arm::JointInertias rotors{ 0.001f, 0.001f, 0.001f, 0.0001f, 0.0001f, 0.0001f };
        arm::DrivenArm drivenArm{ model, rotors };
        arm::SlotineLi controller{ drivenArm, arm::SlotineLiTuning{}, period };
        arm::MeasuredJoints measured{};
    };
}

TEST_F(SlotineLiTest, holds_the_reference_at_rest_with_the_gravity_torque_from_the_models_parameters)
{
    EXPECT_TRUE(controller.BumplessTransfer());
    controller.Reset(measured, arm::AtRest(measured.positions));
    const auto setpoints = controller.Step(measured, arm::AtRest(measured.positions));
    const auto gravity = model.GravityTorque(measured.positions);

    EXPECT_EQ(measured.positions, setpoints.positions);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(gravity[joint].Value(), setpoints.torques[joint].Value(), tolerance) << "joint " << joint;
    EXPECT_TRUE(Same(arm::SlotineLi::Regressor::Parameters(model.Links()), controller.Estimate()));
}

TEST_F(SlotineLiTest, on_the_reference_it_gives_the_models_inverse_dynamics_with_the_rotors)
{
    auto reference = arm::AtRest(measured.positions);
    reference.speeds[1] = foc::RadiansPerSecond{ 0.5f };
    reference.accelerations[2] = foc::RadiansPerSecondSquared{ 2.0f };
    measured.speeds = reference.speeds;

    const auto torques = controller.Step(measured, reference).torques;
    const auto forward = model.InverseDynamics(measured.positions, reference.speeds, reference.accelerations);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(forward[joint].Value() + rotors[joint] * reference.accelerations[joint].Value(), torques[joint].Value(), tolerance) << "joint " << joint;
}

TEST_F(SlotineLiTest, learns_a_payload_the_model_does_not_know_and_tracks_closer_than_without_adaptation)
{
    const auto fixed = TrackingError(1.0e-9f);
    const auto adapted = TrackingError(10.0f);

    EXPECT_LT(adapted, 0.5f * fixed) << "fixed " << fixed << " rad, adapted " << adapted << " rad";
}

TEST_F(SlotineLiTest, keeps_what_it_learnt_across_a_reset_and_returns_to_the_models_parameters_on_demand)
{
    arm::SlotineLi learning{ drivenArm, arm::SlotineLiTuning{}, period };
    auto reference = arm::AtRest(measured.positions);
    reference.positions[1] = foc::Radians{ reference.positions[1].Value() + 0.1f };
    learning.Step(measured, reference);
    const auto learnt = learning.Estimate();
    EXPECT_FALSE(Same(arm::SlotineLi::Regressor::Parameters(model.Links()), learnt));

    learning.Reset(measured, arm::AtRest(measured.positions));
    EXPECT_TRUE(Same(learnt, learning.Estimate()));

    learning.ResetEstimate();
    EXPECT_TRUE(Same(arm::SlotineLi::Regressor::Parameters(model.Links()), learning.Estimate()));
}
