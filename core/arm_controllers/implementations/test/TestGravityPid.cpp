#include "core/arm_controllers/implementations/GravityPid.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    using namespace std::chrono_literals;

    constexpr float tolerance{ 1.0e-3f };

    arm::JointLimits WideLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        return limits;
    }

    class GravityPidTest
        : public testing::Test
    {
    public:
        GravityPidTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                measured.positions[joint] = foc::Radians{ 0.1f * static_cast<float>(joint) };
            controller.Reset(measured, arm::AtRest(measured.positions));
        }

        arm::JointTorques Expected(const arm::JointAccelerations& accelerations) const
        {
            auto torques = model.GravityTorque(measured.positions);
            auto inertia = model.JointMassMatrix(arm::JointPositions{});
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                inertia.at(joint, joint) += rotors[joint];
            for (std::size_t row = 0; row != arm::jointCount; ++row)
                for (std::size_t column = 0; column != arm::jointCount; ++column)
                    torques[row] = foc::NewtonMeter{ torques[row].Value() + inertia.at(row, column) * accelerations[column].Value() };
            return torques;
        }

        void ExpectTorques(const arm::JointTorques& expected, const arm::JointTorques& actual) const
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                EXPECT_NEAR(expected[joint].Value(), actual[joint].Value(), tolerance) << "joint " << joint;
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        arm::JointInertias rotors{ 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f };
        arm::MeasuredJoints measured{};
        arm::GravityPid controller{ model, rotors, arm::BandwidthTuning{ 10.0f, 0.5f }, 10ms };
    };
}

TEST_F(GravityPidTest, holds_a_joint_at_rest_on_its_reference_with_the_gravity_torque_alone)
{
    const auto setpoints = controller.Step(measured, arm::AtRest(measured.positions));

    EXPECT_EQ(measured.positions, setpoints.positions);
    ExpectTorques(model.GravityTorque(measured.positions), setpoints.torques);
}

TEST_F(GravityPidTest, feeds_the_reference_acceleration_forward_through_the_arms_inertia)
{
    auto reference = arm::AtRest(measured.positions);
    reference.accelerations[1] = foc::RadiansPerSecondSquared{ 2.0f };

    ExpectTorques(Expected(reference.accelerations), controller.Step(measured, reference).torques);
}

TEST_F(GravityPidTest, integrates_a_held_position_error_period_after_period_on_top_of_its_proportional_kick)
{
    auto reference = arm::AtRest(measured.positions);
    reference.positions[2] = foc::Radians{ reference.positions[2].Value() + 0.01f };

    const auto first = controller.Step(measured, reference).torques;
    const auto second = controller.Step(measured, reference).torques;

    arm::JointAccelerations correction{};
    correction[2] = foc::RadiansPerSecondSquared{ (200.0f + 1000.0f * 0.01f) * 0.01f };
    ExpectTorques(Expected(correction), first);
    correction[2] = foc::RadiansPerSecondSquared{ (200.0f + 2.0f * 1000.0f * 0.01f) * 0.01f };
    ExpectTorques(Expected(correction), second);
}

TEST_F(GravityPidTest, a_new_position_error_acts_through_the_proportional_gain_alone_while_the_speed_holds)
{
    controller.Step(measured, arm::AtRest(measured.positions));
    auto reference = arm::AtRest(measured.positions);
    reference.positions[2] = foc::Radians{ reference.positions[2].Value() + 0.01f };

    arm::JointAccelerations correction{};
    correction[2] = foc::RadiansPerSecondSquared{ (200.0f + 1000.0f * 0.01f) * 0.01f };
    ExpectTorques(Expected(correction), controller.Step(measured, reference).torques);
}

TEST_F(GravityPidTest, damps_the_measured_speed_error_through_the_derivative_gain)
{
    auto reference = arm::AtRest(measured.positions);
    reference.speeds[2] = foc::RadiansPerSecond{ 0.5f };
    measured.speeds[2] = foc::RadiansPerSecond{ 0.1f };

    arm::JointAccelerations correction{};
    correction[2] = foc::RadiansPerSecondSquared{ 20.0f * 0.4f };
    ExpectTorques(Expected(correction), controller.Step(measured, reference).torques);
}

TEST_F(GravityPidTest, clamps_each_joints_correction_to_200_rad_per_s2)
{
    controller.Step(measured, arm::AtRest(measured.positions));
    auto reference = arm::AtRest(measured.positions);
    reference.positions[0] = foc::Radians{ reference.positions[0].Value() + 2.0f };
    reference.speeds[1] = foc::RadiansPerSecond{ -20.0f };

    arm::JointAccelerations clamped{};
    clamped[0] = foc::RadiansPerSecondSquared{ 200.0f };
    clamped[1] = foc::RadiansPerSecondSquared{ -200.0f };
    ExpectTorques(Expected(clamped), controller.Step(measured, reference).torques);
}

TEST_F(GravityPidTest, takes_over_an_error_without_a_step_from_a_fresh_integral)
{
    auto reference = arm::AtRest(measured.positions);
    reference.positions[2] = foc::Radians{ reference.positions[2].Value() + 0.01f };
    controller.Step(measured, reference);
    controller.Step(measured, reference);

    controller.Reset(measured, reference);

    EXPECT_TRUE(controller.BumplessTransfer());
    arm::JointAccelerations integral{};
    integral[2] = foc::RadiansPerSecondSquared{ 1000.0f * 0.01f * 0.01f };
    ExpectTorques(Expected(integral), controller.Step(measured, reference).torques);
}

TEST_F(GravityPidTest, a_reference_step_in_the_first_period_after_a_switch_keeps_its_proportional_kick)
{
    auto reference = arm::AtRest(measured.positions);
    reference.positions[2] = foc::Radians{ reference.positions[2].Value() + 0.05f };

    arm::JointAccelerations kick{};
    kick[2] = foc::RadiansPerSecondSquared{ (200.0f + 1000.0f * 0.01f) * 0.05f };
    ExpectTorques(Expected(kick), controller.Step(measured, reference).torques);
}
