#include "core/joint_bus/interfaces/JointTransmission.hpp"
#include "gmock/gmock.h"
#include <limits>

namespace
{
    constexpr float tolerance{ 1.0e-5f };

    class JointTransmissionTest
        : public testing::Test
    {
    public:
        arm::JointTransmission geared{ 10.0f, arm::JointDirection::negative, foc::Radians{ 0.5f } };
    };
}

TEST_F(JointTransmissionTest, an_identity_transmission_leaves_angles_and_speeds_unchanged)
{
    const arm::JointTransmission identity{};

    EXPECT_NEAR(0.7f, identity.MotorPosition(foc::Radians{ 0.7f }).Value(), tolerance);
    EXPECT_NEAR(-1.2f, identity.JointSpeed(foc::RadiansPerSecond{ -1.2f }).Value(), tolerance);
}

TEST_F(JointTransmissionTest, the_motor_turns_ratio_times_the_joint_from_the_offset_in_the_configured_direction)
{
    EXPECT_NEAR(-4.0f, geared.MotorPosition(foc::Radians{ 0.9f }).Value(), tolerance);
    EXPECT_NEAR(0.0f, geared.MotorPosition(foc::Radians{ 0.5f }).Value(), tolerance);
    EXPECT_NEAR(-3.0f, geared.MotorSpeed(foc::RadiansPerSecond{ 0.3f }).Value(), tolerance);
}

TEST_F(JointTransmissionTest, a_motor_reading_converts_back_to_the_joint)
{
    EXPECT_NEAR(0.9f, geared.JointPosition(foc::Radians{ -4.0f }).Value(), tolerance);
    EXPECT_NEAR(0.3f, geared.JointSpeed(foc::RadiansPerSecond{ -3.0f }).Value(), tolerance);
}

TEST_F(JointTransmissionTest, joint_to_motor_and_back_round_trips)
{
    for (const auto joint : { -2.5f, -0.1f, 0.0f, 0.5f, 3.0f })
        EXPECT_NEAR(joint, geared.JointPosition(geared.MotorPosition(foc::Radians{ joint })).Value(), tolerance) << joint;
}

TEST_F(JointTransmissionTest, a_transmission_needs_a_positive_finite_ratio_and_a_finite_offset)
{
    EXPECT_TRUE(geared.Valid());
    EXPECT_FALSE((arm::JointTransmission{ 0.0f, arm::JointDirection::positive, foc::Radians{ 0.0f } }.Valid()));
    EXPECT_FALSE((arm::JointTransmission{ -2.0f, arm::JointDirection::positive, foc::Radians{ 0.0f } }.Valid()));
    EXPECT_FALSE((arm::JointTransmission{ 2.0f, arm::JointDirection::positive, foc::Radians{ std::numeric_limits<float>::infinity() } }.Valid()));
}
