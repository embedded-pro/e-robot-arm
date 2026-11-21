#include "core/arm_model/DrivenArm.hpp"
#include "gmock/gmock.h"

namespace
{
    constexpr float tolerance{ 1.0e-5f };

    arm::JointLimits WideLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        return limits;
    }

    arm::ArmGeometry SignedGeometry()
    {
        auto geometry = arm::ReferenceGeometry();
        geometry.opw.signs = { 1, -1, 1, -1, 1, -1 };
        geometry.opw.offsets = { 0.1f, -0.2f, 0.3f, 0.0f, 0.2f, -0.1f };
        return geometry;
    }

    arm::JointPositions Pose()
    {
        return { foc::Radians{ 0.2f }, foc::Radians{ -0.4f }, foc::Radians{ 0.6f }, foc::Radians{ 0.1f }, foc::Radians{ 0.5f }, foc::Radians{ -0.3f } };
    }

    class DrivenArmTest
        : public testing::Test
    {
    public:
        arm::ArmModel model{ SignedGeometry(), arm::ReferenceMass(), WideLimits() };
        const arm::JointInertias rotors{ 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f };
        arm::DrivenArm driven{ model, rotors };
    };
}

TEST_F(DrivenArmTest, keeps_the_arm_model_and_the_rotors_it_was_given)
{
    EXPECT_EQ(&driven.Model(), &model);
    EXPECT_EQ(driven.Rotors(), rotors);
}

TEST_F(DrivenArmTest, puts_each_rotor_on_its_link_as_armature)
{
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_FLOAT_EQ(driven.Links()[joint].armature, rotors[joint]);
        EXPECT_FLOAT_EQ(driven.Links()[joint].mass, model.Links()[joint].mass);
    }
}

TEST_F(DrivenArmTest, adds_each_rotor_to_the_diagonal_of_the_mass_matrix)
{
    const auto angles = model.ModelVector(Pose());
    const auto arm = model.Dynamics().ComputeMassMatrix(angles);
    const auto driving = driven.MassMatrix(angles);

    for (std::size_t row = 0; row != arm::jointCount; ++row)
        for (std::size_t column = 0; column != arm::jointCount; ++column)
            EXPECT_NEAR(driving.at(row, column), arm.at(row, column) + (row == column ? rotors[row] : 0.0f), tolerance);
}

TEST_F(DrivenArmTest, adds_each_rotor_to_the_diagonal_of_the_mass_matrix_in_joint_coordinates)
{
    const auto arm = model.JointMassMatrix(Pose());
    const auto driving = driven.JointMassMatrix(Pose());

    for (std::size_t row = 0; row != arm::jointCount; ++row)
        for (std::size_t column = 0; column != arm::jointCount; ++column)
            EXPECT_NEAR(driving.at(row, column), arm.at(row, column) + (row == column ? rotors[row] : 0.0f), tolerance);
}

TEST_F(DrivenArmTest, adds_each_rotors_inertia_times_its_acceleration_to_the_inverse_dynamics)
{
    const auto angles = model.ModelVector(Pose());
    const arm::ArmModel::JointVector speeds{ 0.3f, -0.2f, 0.1f, 0.4f, -0.5f, 0.2f };
    const arm::ArmModel::JointVector accelerations{ 1.0f, -2.0f, 0.5f, 3.0f, -1.5f, 2.5f };
    const auto arm = model.InverseModel().ComputeInverseDynamics(angles, speeds, accelerations);
    const auto driving = driven.LinkDynamics().ComputeInverseDynamics(angles, speeds, accelerations);

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(driving.at(joint, 0), arm.at(joint, 0) + rotors[joint] * accelerations.at(joint, 0), tolerance);
}

TEST_F(DrivenArmTest, leaves_the_gravity_and_coriolis_terms_to_the_links)
{
    const auto angles = model.ModelVector(Pose());
    const arm::ArmModel::JointVector speeds{ 0.3f, -0.2f, 0.1f, 0.4f, -0.5f, 0.2f };
    const auto gravity = model.Dynamics().ComputeGravityTerms(angles);
    const auto coriolis = model.Dynamics().ComputeCoriolisTerms(angles, speeds);

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_NEAR(driven.LinkDynamics().ComputeGravityTerms(angles).at(joint, 0), gravity.at(joint, 0), tolerance);
        EXPECT_NEAR(driven.LinkDynamics().ComputeCoriolisTerms(angles, speeds).at(joint, 0), coriolis.at(joint, 0), tolerance);
    }
}
