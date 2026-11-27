#include "core/arm/ArmControllerTuning.hpp"
#include "gmock/gmock.h"
#include <array>
#include <limits>
#include <utility>

namespace
{
    using arm::ArmControllerKind;
    using arm::TuningField;
    using arm::TuningViolation;
}

TEST(ArmControllerTuningTest, every_controller_but_the_passive_ones_is_tunable_with_its_own_item)
{
    EXPECT_TRUE(arm::Tunable(ArmControllerKind::gravityPid));
    EXPECT_FALSE(arm::Tunable(ArmControllerKind::free));
    EXPECT_FALSE(arm::Tunable(ArmControllerKind::gravityFloat));

    EXPECT_TRUE(arm::TuningFits(ArmControllerKind::computedTorque, arm::BandwidthTuning{}));
    EXPECT_TRUE(arm::TuningFits(ArmControllerKind::slotineLi, arm::SlotineLiTuning{}));
    EXPECT_FALSE(arm::TuningFits(ArmControllerKind::gravityPd, arm::SlotineLiTuning{}));
    EXPECT_FALSE(arm::TuningFits(ArmControllerKind::free, arm::BandwidthTuning{}));
}

TEST(ArmControllerTuningTest, accepts_the_defaults_and_names_the_first_field_outside_its_range)
{
    EXPECT_EQ(std::nullopt, arm::CheckTuning(arm::BandwidthTuning{}));
    EXPECT_EQ(std::nullopt, arm::CheckTuning(arm::JointImpedanceTuning{}));
    EXPECT_EQ(std::nullopt, arm::CheckTuning(arm::CartesianImpedanceTuning{}));
    EXPECT_EQ(std::nullopt, arm::CheckTuning(arm::OperationalSpaceTuning{}));
    EXPECT_EQ(std::nullopt, arm::CheckTuning(arm::SlotineLiTuning{}));
    EXPECT_EQ(std::nullopt, arm::CheckTuning(arm::BandwidthTuning{ 500.0f, 5.0f }));

    EXPECT_EQ((TuningViolation{ TuningField::bandwidth, 500.0f }), arm::CheckTuning(arm::BandwidthTuning{ 501.0f, 0.7f }));
    EXPECT_EQ((TuningViolation{ TuningField::dampingRatio, 5.0f }), arm::CheckTuning(arm::BandwidthTuning{ 30.0f, 0.0f }));
    EXPECT_EQ((TuningViolation{ TuningField::bandwidth, 500.0f }), arm::CheckTuning(arm::BandwidthTuning{ std::numeric_limits<float>::quiet_NaN(), 0.7f }));
    arm::JointStiffnesses stiffness{};
    stiffness.fill(10.0f);
    stiffness[3] = -1.0f;
    EXPECT_EQ((TuningViolation{ TuningField::stiffness, std::nullopt }), arm::CheckTuning(arm::JointImpedanceTuning{ stiffness, 0.7f }));
    EXPECT_EQ((TuningViolation{ TuningField::rotationalStiffness, std::nullopt }), arm::CheckTuning(arm::CartesianImpedanceTuning{ 50.0f, 0.0f, 0.7f }));
    EXPECT_EQ((TuningViolation{ TuningField::singularityDamping, 1.0f }), arm::CheckTuning(arm::OperationalSpaceTuning{ 30.0f, 0.7f, 1.5f }));
    EXPECT_EQ((TuningViolation{ TuningField::adaptationGain, 10000.0f }), arm::CheckTuning(arm::SlotineLiTuning{ 10.0f, 20.0f, 20000.0f }));
}

TEST(ArmControllerTuningTest, stores_a_tuning_in_the_configuration_item_of_its_controller)
{
    arm::ArmConfig config{};
    arm::StoreTuning(config, ArmControllerKind::gravityPid, arm::BandwidthTuning{ 12.0f, 0.9f });
    arm::StoreTuning(config, ArmControllerKind::operationalSpace, arm::OperationalSpaceTuning{ 25.0f, 0.8f, 0.02f });
    arm::StoreTuning(config, ArmControllerKind::gravityPd, arm::SlotineLiTuning{});

    EXPECT_FLOAT_EQ(12.0f, config.gravityPid.bandwidth);
    EXPECT_EQ((arm::OperationalSpaceTuning{ 25.0f, 0.8f, 0.02f }), config.operationalSpace);
    EXPECT_FLOAT_EQ(30.0f, config.gravityPd.bandwidth);
    EXPECT_FLOAT_EQ(12.0f, std::get<arm::BandwidthTuning>(arm::TuningOf(config, ArmControllerKind::gravityPid)).bandwidth);
}

TEST(ArmControllerTuningTest, every_tunable_controller_round_trips_its_tuning_through_the_configuration)
{
    const std::array<std::pair<ArmControllerKind, arm::ArmControllerTuning>, 7> tunings{ {
        { ArmControllerKind::gravityPd, arm::BandwidthTuning{ 11.0f, 0.6f } },
        { ArmControllerKind::computedTorque, arm::BandwidthTuning{ 12.0f, 0.6f } },
        { ArmControllerKind::gravityPid, arm::BandwidthTuning{ 13.0f, 0.6f } },
        { ArmControllerKind::jointImpedance, arm::JointImpedanceTuning{ std::nullopt, 0.6f } },
        { ArmControllerKind::cartesianImpedance, arm::CartesianImpedanceTuning{ 40.0f, 0.05f, 0.6f } },
        { ArmControllerKind::operationalSpace, arm::OperationalSpaceTuning{ 14.0f, 0.6f, 0.02f } },
        { ArmControllerKind::slotineLi, arm::SlotineLiTuning{ 5.0f, 15.0f, 20.0f } },
    } };

    arm::ArmConfig config{};
    for (const auto& [kind, tuning] : tunings)
        arm::StoreTuning(config, kind, tuning);
    arm::StoreTuning(config, ArmControllerKind::free, arm::BandwidthTuning{});

    for (const auto& [kind, tuning] : tunings)
        EXPECT_EQ(tuning.index(), arm::TuningOf(config, kind).index()) << arm::ArmControllerName(kind);
    EXPECT_FLOAT_EQ(11.0f, config.gravityPd.bandwidth);
    EXPECT_FLOAT_EQ(12.0f, config.computedTorque.bandwidth);
    EXPECT_FLOAT_EQ(0.6f, config.jointImpedance.dampingRatio);
    EXPECT_EQ((arm::CartesianImpedanceTuning{ 40.0f, 0.05f, 0.6f }), config.cartesianImpedance);
    EXPECT_EQ((arm::SlotineLiTuning{ 5.0f, 15.0f, 20.0f }), config.slotineLi);
    EXPECT_TRUE(arm::TuningFits(ArmControllerKind::jointImpedance, arm::TuningOf(config, ArmControllerKind::jointImpedance)));
    EXPECT_TRUE(arm::TuningFits(ArmControllerKind::cartesianImpedance, arm::TuningOf(config, ArmControllerKind::cartesianImpedance)));
    EXPECT_TRUE(arm::TuningFits(ArmControllerKind::operationalSpace, arm::TuningOf(config, ArmControllerKind::operationalSpace)));
}

TEST(ArmControllerTuningTest, saves_a_controllers_current_tuning_as_a_preset_and_replaces_one_of_the_same_name)
{
    arm::ArmConfig config{};
    config.gravityPd = { 12.0f, 0.9f };

    EXPECT_EQ(arm::PresetResult::saved, arm::SavePreset(config, ArmControllerKind::gravityPd, "soft"));
    EXPECT_EQ(arm::PresetResult::saved, arm::SavePreset(config, ArmControllerKind::gravityPid, "soft"));
    ASSERT_EQ(2u, config.presetCount);
    EXPECT_FLOAT_EQ(12.0f, std::get<arm::BandwidthTuning>(config.presets[0].tuning).bandwidth);

    config.gravityPd = { 20.0f, 0.7f };
    EXPECT_EQ(arm::PresetResult::replaced, arm::SavePreset(config, ArmControllerKind::gravityPd, "soft"));
    EXPECT_EQ(2u, config.presetCount);
    EXPECT_FLOAT_EQ(20.0f, std::get<arm::BandwidthTuning>(config.presets[0].tuning).bandwidth);

    EXPECT_EQ(arm::PresetResult::notTunable, arm::SavePreset(config, ArmControllerKind::free, "soft"));
    EXPECT_EQ(arm::PresetResult::badName, arm::SavePreset(config, ArmControllerKind::gravityPd, ""));
    EXPECT_EQ(arm::PresetResult::badName, arm::SavePreset(config, ArmControllerKind::gravityPd, "seventeen chars!!"));
}

TEST(ArmControllerTuningTest, refuses_a_seventeenth_preset)
{
    arm::ArmConfig config{};
    const std::array<const char*, arm::maxArmControllerPresets> names{ "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p" };
    for (const auto* name : names)
        EXPECT_EQ(arm::PresetResult::saved, arm::SavePreset(config, ArmControllerKind::gravityPd, name));

    EXPECT_EQ(arm::PresetResult::full, arm::SavePreset(config, ArmControllerKind::gravityPd, "q"));
    EXPECT_EQ(arm::PresetResult::replaced, arm::SavePreset(config, ArmControllerKind::gravityPd, "p"));
}
