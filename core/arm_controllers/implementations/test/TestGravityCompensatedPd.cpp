#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"
#include "gmock/gmock.h"

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

    class GravityCompensatedPdTest
        : public testing::Test
    {
    public:
        GravityCompensatedPdTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                measured.positions[joint] = foc::Radians{ 0.1f * static_cast<float>(joint) };
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        const arm::JointInertias rotors{ 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f };
        arm::GravityCompensatedPd controller{ model, rotors, arm::BandwidthTuning{ 20.0f, 0.5f } };
        arm::MeasuredJoints measured{};
    };
}

TEST_F(GravityCompensatedPdTest, holds_a_joint_at_rest_on_its_reference_with_the_gravity_torque_alone)
{
    controller.Reset(measured, arm::AtRest(measured.positions));
    const auto setpoints = controller.Step(measured, arm::AtRest(measured.positions));
    const auto gravity = model.GravityTorque(measured.positions);

    EXPECT_EQ(measured.positions, setpoints.positions);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(gravity[joint].Value(), setpoints.torques[joint].Value(), tolerance) << "joint " << joint;
    EXPECT_GT(std::abs(gravity[1].Value()), 1.0f);
}

TEST_F(GravityCompensatedPdTest, places_the_stiffness_and_damping_through_the_inertia_at_the_measured_pose)
{
    auto reference = measured.positions;
    reference[2] = foc::Radians{ reference[2].Value() + 0.05f };
    measured.speeds[4] = foc::RadiansPerSecond{ 2.0f };

    const auto torques = controller.Step(measured, arm::AtRest(reference)).torques;
    const auto gravity = model.GravityTorque(measured.positions);

    auto inertia = model.JointMassMatrix(measured.positions);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        inertia.at(joint, joint) += rotors[joint];
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(gravity[joint].Value() + 400.0f * inertia.at(joint, 2) * 0.05f - 20.0f * inertia.at(joint, 4) * 2.0f, torques[joint].Value(), tolerance) << "joint " << joint;
}

TEST_F(GravityCompensatedPdTest, takes_over_without_a_step)
{
    EXPECT_TRUE(controller.BumplessTransfer());
}

TEST_F(GravityCompensatedPdTest, places_every_mode_at_the_bandwidth_through_the_inertia_matrix_at_a_pose)
{
    const auto gains = arm::PlacedGains(model, rotors, arm::BandwidthTuning{ 30.0f, 0.7f }, measured.positions);

    auto inertia = model.Dynamics().ComputeMassMatrix(model.ModelVector(measured.positions));
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        inertia.at(joint, joint) += rotors[joint];
    for (std::size_t row = 0; row != arm::jointCount; ++row)
        for (std::size_t column = 0; column != arm::jointCount; ++column)
        {
            EXPECT_NEAR(900.0f * inertia.at(row, column), gains.kp.at(row, column), 1.0e-3f);
            EXPECT_NEAR(42.0f * inertia.at(row, column), gains.kd.at(row, column), 1.0e-4f);
        }
}
