#include "core/arm_controllers/implementations/ComputedTorque.hpp"
#include "gmock/gmock.h"

namespace
{
    constexpr float tolerance{ 1.0e-3f };

    arm::JointLimits WideLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        return limits;
    }

    class ComputedTorqueTest
        : public testing::Test
    {
    public:
        ComputedTorqueTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                measured.positions[joint] = foc::Radians{ 0.1f * static_cast<float>(joint) };
        }

        arm::JointTorques Expected(const arm::JointAccelerations& accelerations) const
        {
            auto torques = model.InverseDynamics(measured.positions, measured.speeds, accelerations);
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                torques[joint] = foc::NewtonMeter{ torques[joint].Value() + rotors[joint] * accelerations[joint].Value() };
            return torques;
        }

        void ExpectTorques(const arm::JointTorques& expected, const arm::JointTorques& actual) const
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                EXPECT_NEAR(expected[joint].Value(), actual[joint].Value(), tolerance) << "joint " << joint;
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        arm::JointInertias rotors{ 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f };
        arm::ComputedTorque controller{ model, rotors, arm::BandwidthTuning{ 20.0f, 0.5f } };
        arm::MeasuredJoints measured{};
    };
}

TEST_F(ComputedTorqueTest, holds_a_joint_at_rest_on_its_reference_with_the_gravity_torque_alone)
{
    const auto setpoints = controller.Step(measured, arm::AtRest(measured.positions));

    EXPECT_EQ(measured.positions, setpoints.positions);
    ExpectTorques(model.GravityTorque(measured.positions), setpoints.torques);
}

TEST_F(ComputedTorqueTest, gives_the_reference_acceleration_through_the_arms_and_the_rotors_inertia)
{
    auto reference = arm::AtRest(measured.positions);
    reference.accelerations[1] = foc::RadiansPerSecondSquared{ 2.0f };
    reference.accelerations[4] = foc::RadiansPerSecondSquared{ -1.0f };

    ExpectTorques(Expected(reference.accelerations), controller.Step(measured, reference).torques);
}

TEST_F(ComputedTorqueTest, corrects_the_position_and_speed_errors_in_acceleration)
{
    auto reference = arm::AtRest(measured.positions);
    reference.positions[2] = foc::Radians{ reference.positions[2].Value() + 0.01f };
    reference.speeds[3] = foc::RadiansPerSecond{ 0.5f };

    arm::JointAccelerations corrections{};
    corrections[2] = foc::RadiansPerSecondSquared{ 400.0f * 0.01f };
    corrections[3] = foc::RadiansPerSecondSquared{ 2.0f * 0.5f * 20.0f * 0.5f };

    ExpectTorques(Expected(corrections), controller.Step(measured, reference).torques);
}

TEST_F(ComputedTorqueTest, takes_over_without_a_step)
{
    EXPECT_TRUE(controller.BumplessTransfer());
}
