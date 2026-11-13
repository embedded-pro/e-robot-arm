#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "core/joint_configurator/interfaces/MotorSettings.hpp"
#include "gmock/gmock.h"

namespace
{
    class ControllerSettingsTest
        : public testing::Test
    {
    public:
        arm::ControllerSettings applied{};
    };
}

TEST_F(ControllerSettingsTest, equal_settings_have_no_mismatch)
{
    EXPECT_FALSE(arm::Compare(applied, applied).Any());
}

TEST_F(ControllerSettingsTest, names_every_algorithm_and_gain_that_differs)
{
    auto reported = applied;
    reported.speed = can::FocSpeedAlgorithm::adrc;
    reported[arm::Gain::positionBandwidth] = 25.0f;

    const auto mismatches = arm::Compare(applied, reported);

    EXPECT_TRUE(mismatches.Any());
    EXPECT_FALSE(mismatches.current);
    EXPECT_TRUE(mismatches.speed);
    EXPECT_TRUE(mismatches.gains[static_cast<std::size_t>(arm::Gain::positionBandwidth)]);
    EXPECT_FALSE(mismatches.gains[static_cast<std::size_t>(arm::Gain::speedBandwidth)]);
}

TEST_F(ControllerSettingsTest, a_gain_within_the_relative_tolerance_matches)
{
    auto reported = applied;
    reported[arm::Gain::currentBandwidth] = applied[arm::Gain::currentBandwidth] * (1.0f + 5.0e-5f);
    EXPECT_FALSE(arm::Compare(applied, reported).Any());

    reported[arm::Gain::currentBandwidth] = applied[arm::Gain::currentBandwidth] * (1.0f + 5.0e-4f);
    EXPECT_TRUE(arm::Compare(applied, reported).Any());
}

TEST_F(ControllerSettingsTest, each_algorithm_uses_its_own_gains_and_the_bandwidth_of_its_loop)
{
    EXPECT_TRUE(arm::Uses(applied, arm::Gain::speedBandwidth));
    EXPECT_FALSE(arm::Uses(applied, arm::Gain::speedErrorWeight));

    applied.current = can::FocCurrentAlgorithm::slidingMode;
    applied.speed = can::FocSpeedAlgorithm::lqi;
    applied.position = can::FocPositionAlgorithm::lqr;

    EXPECT_TRUE(arm::Uses(applied, arm::Gain::currentSwitchingGain));
    EXPECT_FALSE(arm::Uses(applied, arm::Gain::currentTwoStepDeadbeat));
    EXPECT_TRUE(arm::Uses(applied, arm::Gain::speedIntegralWeight));
    EXPECT_TRUE(arm::Uses(applied, arm::Gain::positionSpeedErrorWeight));
    EXPECT_FALSE(arm::Uses(applied, arm::Gain::positionIntegralWeight));
    EXPECT_FALSE(arm::Uses(applied, arm::Gain::positionReferenceTimeConstant));
}

TEST_F(ControllerSettingsTest, motor_values_compare_relative_to_their_size_even_when_tiny)
{
    auto actual = arm::motorPresets[0].settings;
    EXPECT_FALSE(arm::Compare(arm::motorPresets[0].settings, actual).Any());

    actual[arm::MotorParameter::inertia] *= 1.001f;
    const auto mismatches = arm::Compare(arm::motorPresets[0].settings, actual);
    EXPECT_TRUE(mismatches.Any());
    EXPECT_TRUE(mismatches.values[static_cast<std::size_t>(arm::MotorParameter::inertia)]);
    EXPECT_FALSE(mismatches.values[static_cast<std::size_t>(arm::MotorParameter::resistance)]);
}
