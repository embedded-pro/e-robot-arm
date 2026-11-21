#include "core/arm_controllers/implementations/JointImpedance.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "gmock/gmock.h"
#include <cmath>

namespace
{
    constexpr float tolerance{ 1.0e-4f };

    arm::JointLimits WideLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        return limits;
    }

    arm::JointImpedanceTuning Tuning()
    {
        return { arm::JointStiffnesses{ 40.0f, 80.0f, 60.0f, 10.0f, 8.0f, 4.0f }, 0.5f };
    }

    class JointImpedanceTest
        : public testing::Test
    {
    public:
        JointImpedanceTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                measured.positions[joint] = foc::Radians{ 0.1f * static_cast<float>(joint) };
        }

        arm::ArmModel::MassMatrix Inertia() const
        {
            auto inertia = model.JointMassMatrix(measured.positions);
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                inertia.at(joint, joint) += rotors[joint];
            return inertia;
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        const arm::JointInertias rotors{ 0.01f, 0.01f, 0.01f, 0.001f, 0.001f, 0.001f };
        arm::DrivenArm drivenArm{ model, rotors };
        arm::JointImpedance controller{ drivenArm, Tuning() };
        arm::MeasuredJoints measured{};
    };
}

TEST_F(JointImpedanceTest, holds_the_reference_at_rest_with_the_gravity_torque_alone_and_takes_over_without_a_step)
{
    EXPECT_TRUE(controller.BumplessTransfer());
    controller.Reset(measured, arm::AtRest(measured.positions));
    const auto setpoints = controller.Step(measured, arm::AtRest(measured.positions));
    const auto gravity = model.GravityTorque(measured.positions);

    EXPECT_EQ(measured.positions, setpoints.positions);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(gravity[joint].Value(), setpoints.torques[joint].Value(), tolerance) << "joint " << joint;
}

TEST_F(JointImpedanceTest, feeds_the_references_inverse_dynamics_forward_so_an_exact_model_tracks_without_error)
{
    auto reference = arm::AtRest(measured.positions);
    reference.speeds[1] = foc::RadiansPerSecond{ 0.4f };
    reference.accelerations[2] = foc::RadiansPerSecondSquared{ 1.5f };
    measured.speeds[1] = foc::RadiansPerSecond{ 0.4f };

    const auto torques = controller.Step(measured, reference).torques;
    const auto forward = model.InverseDynamics(measured.positions, reference.speeds, reference.accelerations);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(forward[joint].Value(), torques[joint].Value(), tolerance) << "joint " << joint;
}

TEST_F(JointImpedanceTest, pulls_each_joint_with_its_stiffness_and_damps_with_the_coupled_inertia_at_the_measured_pose)
{
    const auto inertia = Inertia();
    const auto damping = controller.Damping(measured.positions);
    const auto stiffness = Tuning().stiffness.value();
    for (std::size_t row = 0; row != arm::jointCount; ++row)
        for (std::size_t column = 0; column != arm::jointCount; ++column)
        {
            const auto weight = std::sqrt(std::sqrt(stiffness[row] / inertia.at(row, row)) * std::sqrt(stiffness[column] / inertia.at(column, column)));
            EXPECT_NEAR(2.0f * 0.5f * weight * inertia.at(row, column), damping.at(row, column), 1.0e-5f) << row << ", " << column;
        }
    EXPECT_NEAR(2.0f * 0.5f * std::sqrt(stiffness[4] * inertia.at(4, 4)), damping.at(4, 4), 1.0e-5f);

    auto reference = arm::AtRest(measured.positions);
    reference.positions[1] = foc::Radians{ reference.positions[1].Value() + 0.05f };
    measured.speeds[4] = foc::RadiansPerSecond{ 2.0f };
    const auto torques = controller.Step(measured, reference).torques;
    const auto forward = model.InverseDynamics(measured.positions, reference.speeds, reference.accelerations);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        const auto spring = joint == 1 ? 80.0f * 0.05f : 0.0f;
        EXPECT_NEAR(forward[joint].Value() + spring - damping.at(joint, 4) * 2.0f, torques[joint].Value(), tolerance) << "joint " << joint;
    }
}

TEST_F(JointImpedanceTest, softens_to_a_share_of_the_profiles_stiffness_and_refuses_one_outside_a_tenth_to_all)
{
    EXPECT_FLOAT_EQ(1.0f, controller.StiffnessShare());
    EXPECT_TRUE(controller.SetStiffnessShare(0.25f));
    EXPECT_FLOAT_EQ(0.25f, controller.StiffnessShare());

    auto reference = arm::AtRest(measured.positions);
    reference.positions[1] = foc::Radians{ reference.positions[1].Value() + 0.05f };
    const auto torques = controller.Step(measured, reference).torques;
    const auto gravity = model.GravityTorque(measured.positions);
    EXPECT_NEAR(gravity[1].Value() + 0.25f * 80.0f * 0.05f, torques[1].Value(), tolerance);
    EXPECT_NEAR(2.0f * 0.5f * std::sqrt(0.25f * 8.0f * Inertia().at(4, 4)), controller.Damping(measured.positions).at(4, 4), 1.0e-5f);

    EXPECT_FALSE(controller.SetStiffnessShare(0.05f));
    EXPECT_FALSE(controller.SetStiffnessShare(1.5f));
    EXPECT_FALSE(controller.SetStiffnessShare(std::nanf("")));
    EXPECT_FLOAT_EQ(0.25f, controller.StiffnessShare());
}

TEST_F(JointImpedanceTest, without_stiffnesses_each_joint_gets_a_ten_radian_per_second_bandwidth_on_its_inertia_with_the_others_free)
{
    const auto home = drivenArm.JointMassMatrix(arm::JointPositions{});
    const auto mobility = solvers::SolveSystem(home, arm::ArmModel::MassMatrix::Identity());
    const auto stiffness = arm::ImpedanceStiffness(home, arm::JointImpedanceTuning{});
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_NEAR(100.0f / mobility.at(joint, joint), stiffness[joint], 1.0e-4f * stiffness[joint]) << "joint " << joint;
        EXPECT_LE(stiffness[joint], 100.0f * home.at(joint, joint) * 1.0001f) << "joint " << joint;
    }
}
