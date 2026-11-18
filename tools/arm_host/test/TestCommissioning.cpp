#include "tools/arm_host/Commissioning.hpp"
#include "tools/arm_host/test/ArmHostTestSupport.hpp"
#include "gmock/gmock.h"

TEST(CommissioningTest, runs_every_joint_at_a_quarter_of_its_speed_and_acceleration_and_trips_at_half)
{
    auto profile = arm_host::test::Config();
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        profile.motionLimits[joint] = { foc::RadiansPerSecond{ 2.0f + joint }, foc::RadiansPerSecondSquared{ 8.0f + joint } };
        profile.collision[joint] = foc::NewtonMeter{ 10.0f + joint };
        profile.tracking[joint] = { foc::Radians{ 0.2f }, std::chrono::milliseconds{ 200 } };
    }

    const auto commissioning = arm_host::CommissioningConfig(profile);

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_FLOAT_EQ(0.25f * (2.0f + joint), commissioning.motionLimits[joint].maxSpeed.Value());
        EXPECT_FLOAT_EQ(0.25f * (8.0f + joint), commissioning.motionLimits[joint].maxAcceleration.Value());
        EXPECT_FLOAT_EQ(0.5f * (10.0f + joint), commissioning.collision[joint].Value());
        EXPECT_FLOAT_EQ(0.1f, commissioning.tracking[joint].window.Value());
        EXPECT_EQ(std::chrono::milliseconds{ 100 }, commissioning.tracking[joint].time);
        EXPECT_EQ(profile.positionLimits[joint].lower.Value(), commissioning.positionLimits[joint].lower.Value());
        EXPECT_EQ(profile.positionLimits[joint].upper.Value(), commissioning.positionLimits[joint].upper.Value());
    }
    EXPECT_EQ(profile.nodes, commissioning.nodes);
}
