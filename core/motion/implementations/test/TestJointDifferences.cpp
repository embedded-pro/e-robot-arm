#include "core/motion/implementations/JointDifferences.hpp"
#include "gmock/gmock.h"

namespace
{
    constexpr float tolerance{ 1.0e-4f };

    class JointDifferencesTest
        : public testing::Test
    {
    public:
        JointDifferencesTest()
        {
            for (auto& limit : limits)
                limit = { foc::RadiansPerSecond{ 2.0f }, foc::RadiansPerSecondSquared{ 50.0f } };
        }

        arm::JointMotionLimits limits{};
        arm::JointPositions previous{ foc::Radians{ 0.1f }, foc::Radians{ 0.2f }, foc::Radians{ 0.3f }, foc::Radians{ 0.4f }, foc::Radians{ 0.5f }, foc::Radians{ 0.6f } };
        arm::JointSpeeds previousSpeeds{ foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecond{ 0.0f }, foc::RadiansPerSecond{ -1.0f }, foc::RadiansPerSecond{ 0.5f },
            foc::RadiansPerSecond{ 0.0f }, foc::RadiansPerSecond{ 0.0f } };
        float period{ 0.01f };
    };
}

TEST_F(JointDifferencesTest, speeds_and_accelerations_are_backward_differences_over_one_period)
{
    arm::JointPositions next{ previous };
    next[0] = foc::Radians{ 0.111f };
    next[2] = foc::Radians{ 0.29f };

    const auto rates = arm::BackwardDifferences(previous, previousSpeeds, next, period);

    EXPECT_NEAR(rates.speeds[0].Value(), 1.1f, tolerance);
    EXPECT_NEAR(rates.accelerations[0].Value(), 10.0f, 1.0e-2f);
    EXPECT_NEAR(rates.speeds[2].Value(), -1.0f, tolerance);
    EXPECT_NEAR(rates.accelerations[2].Value(), 0.0f, 1.0e-2f);
    EXPECT_NEAR(rates.speeds[3].Value(), 0.0f, tolerance);
    EXPECT_NEAR(rates.accelerations[3].Value(), -50.0f, 1.0e-2f);
}

TEST_F(JointDifferencesTest, rates_are_within_the_limits_only_when_every_joint_is)
{
    arm::JointRates rates{};
    rates.speeds[1] = foc::RadiansPerSecond{ -2.0f };
    rates.accelerations[4] = foc::RadiansPerSecondSquared{ 50.0f };
    EXPECT_TRUE(arm::WithinMotionLimits(rates, limits));

    rates.speeds[5] = foc::RadiansPerSecond{ 2.1f };
    EXPECT_FALSE(arm::WithinMotionLimits(rates, limits));

    rates.speeds[5] = foc::RadiansPerSecond{};
    rates.accelerations[0] = foc::RadiansPerSecondSquared{ -50.5f };
    EXPECT_FALSE(arm::WithinMotionLimits(rates, limits));
}
