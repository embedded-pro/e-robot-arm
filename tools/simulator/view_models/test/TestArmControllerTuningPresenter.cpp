#include "tools/simulator/view_models/ArmControllerTuningPresenter.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "ui/backend/recording/RecordingFormView.hpp"
#include "gmock/gmock.h"
#include <string>
#include <vector>

namespace
{
    using simulator::ControllerTuningAction;
    using simulator::ControllerTuningField;
    using simulator::ControllerTuningState;
    using ui::backend::recording::FormCommandKind;

    class ArmControllerTuningPresenterTest
        : public testing::Test
    {
    public:
        ArmControllerTuningPresenterTest()
        {
            snapshot = simulator::test::Running();
            snapshot.armControllers = { { "gravity-pd", true, true }, { "computed-torque", true, false }, { "gravity-pid", true, false },
                { "joint-impedance", true, false }, { "cartesian-impedance", true, false }, { "operational-space", true, false } };
            snapshot.config.gravityPd = { 30.0f, 0.7f };
            presenter.Refresh(snapshot, {});
        }

        bool ActionEnabled(ui::model::ActionId action) const
        {
            bool enabled = true;
            for (const auto& command : view.Commands())
                if (command.kind == FormCommandKind::SetActionEnabled && command.action == action)
                    enabled = command.flag;
            return enabled;
        }

        const arm_host::TuneArmController& Posted() const
        {
            return std::get<arm_host::TuneArmController>(posted.back());
        }

        arm_host::ArmSnapshot snapshot;
        ui::backend::recording::RecordingFormView view;
        std::vector<arm_host::ArmCommand> posted;
        simulator::ArmControllerTuningPresenter presenter{ view, [this](const arm_host::ArmCommand& command)
            {
                posted.push_back(command);
            } };
    };
}

TEST_F(ArmControllerTuningPresenterTest, shows_the_tuning_in_effect_and_offers_nothing_to_apply)
{
    EXPECT_EQ(ControllerTuningState::applied, presenter.State());
    EXPECT_EQ("Applied", presenter.StateText());
    EXPECT_EQ(ui::theme::StatusLevel::Ok, presenter.StateLevel());
    EXPECT_FALSE(ActionEnabled(ControllerTuningAction::apply));
    EXPECT_FALSE(ActionEnabled(ControllerTuningAction::revert));
    EXPECT_EQ((arm::BandwidthTuning{ 30.0f, 0.7f }.bandwidth), std::get<arm::BandwidthTuning>(presenter.FormTuning()).bandwidth);
    EXPECT_TRUE(view.IsControlVisible(ControllerTuningField::bandwidth));
    EXPECT_FALSE(view.IsControlVisible(ControllerTuningField::lambda));
}

TEST_F(ArmControllerTuningPresenterTest, an_edit_is_applied_only_on_apply_and_confirmed_by_the_snapshot)
{
    view.TypeNumber(ControllerTuningField::bandwidth, 20.0);
    EXPECT_EQ(ControllerTuningState::edited, presenter.State());
    EXPECT_EQ("Edited", presenter.StateText());
    EXPECT_TRUE(posted.empty());
    EXPECT_TRUE(ActionEnabled(ControllerTuningAction::apply));

    view.PressAction(ControllerTuningAction::apply);
    ASSERT_EQ(1u, posted.size());
    EXPECT_EQ(arm::ArmControllerKind::gravityPd, Posted().kind);
    EXPECT_FLOAT_EQ(20.0f, std::get<arm::BandwidthTuning>(Posted().tuning).bandwidth);
    EXPECT_EQ("Requested…", presenter.StateText());
    EXPECT_FALSE(ActionEnabled(ControllerTuningAction::apply));

    presenter.Refresh(snapshot, {});
    EXPECT_EQ(ControllerTuningState::requested, presenter.State());
    snapshot.config.gravityPd.bandwidth = 20.0f;
    presenter.Refresh(snapshot, {});
    EXPECT_EQ(ControllerTuningState::applied, presenter.State());
}

TEST_F(ArmControllerTuningPresenterTest, a_refusal_keeps_the_edit_and_shows_the_reason)
{
    view.TypeNumber(ControllerTuningField::dampingRatio, 1.2);
    view.PressAction(ControllerTuningAction::apply);

    const std::vector<arm_host::ArmEvent> events{ { .kind = arm_host::ArmEventKind::refused, .commandName = "tune arm controller", .reason = "moving: wait or abort" } };
    presenter.Refresh(snapshot, events);

    EXPECT_EQ(ControllerTuningState::refused, presenter.State());
    EXPECT_EQ("Refused: moving: wait or abort", presenter.StateText());
    EXPECT_EQ(ui::theme::StatusLevel::Fault, presenter.StateLevel());
    EXPECT_FLOAT_EQ(1.2f, std::get<arm::BandwidthTuning>(presenter.FormTuning()).dampingRatio);
    EXPECT_TRUE(ActionEnabled(ControllerTuningAction::revert));
}

TEST_F(ArmControllerTuningPresenterTest, revert_puts_the_tuning_in_effect_back)
{
    view.TypeNumber(ControllerTuningField::bandwidth, 12.0);
    view.PressAction(ControllerTuningAction::revert);

    EXPECT_EQ(ControllerTuningState::applied, presenter.State());
    EXPECT_FLOAT_EQ(30.0f, std::get<arm::BandwidthTuning>(presenter.FormTuning()).bandwidth);
}

TEST_F(ArmControllerTuningPresenterTest, apply_is_disabled_with_the_reason_while_moving_disconnected_or_not_offered)
{
    view.TypeNumber(ControllerTuningField::bandwidth, 12.0);
    auto moving = snapshot;
    moving.moving = true;
    presenter.Refresh(moving, {});
    EXPECT_FALSE(ActionEnabled(ControllerTuningAction::apply));
    EXPECT_EQ("Apply: Moving: wait or abort", presenter.Hint());

    auto disconnected = snapshot;
    disconnected.targetReady = false;
    presenter.Refresh(disconnected, {});
    EXPECT_EQ("Apply: Not connected", presenter.Hint());

    presenter.Choose(arm::ArmControllerKind::slotineLi);
    view.TypeNumber(ControllerTuningField::lambda, 12.0);
    presenter.Refresh(snapshot, {});
    EXPECT_EQ("Apply: The arm does not offer this controller", presenter.Hint());
    view.PressAction(ControllerTuningAction::apply);
    EXPECT_TRUE(posted.empty());
}

TEST_F(ArmControllerTuningPresenterTest, each_controller_shows_and_sends_its_own_fields)
{
    snapshot.config.operationalSpace = { 25.0f, 0.8f, 0.02f };
    presenter.Refresh(snapshot, {});
    presenter.Choose(arm::ArmControllerKind::operationalSpace);
    EXPECT_TRUE(view.IsControlVisible(ControllerTuningField::singularityDamping));
    EXPECT_EQ((arm::OperationalSpaceTuning{ 25.0f, 0.8f, 0.02f }), std::get<arm::OperationalSpaceTuning>(presenter.FormTuning()));

    presenter.Choose(arm::ArmControllerKind::cartesianImpedance);
    EXPECT_TRUE(view.IsControlVisible(ControllerTuningField::translationalStiffness));
    view.TypeNumber(ControllerTuningField::translationalStiffness, 80.0);
    view.PressAction(ControllerTuningAction::apply);
    EXPECT_EQ(arm::ArmControllerKind::cartesianImpedance, Posted().kind);
    EXPECT_FLOAT_EQ(80.0f, std::get<arm::CartesianImpedanceTuning>(Posted().tuning).translationalStiffness);

    presenter.Choose(arm::ArmControllerKind::jointImpedance);
    EXPECT_TRUE(view.IsControlVisible(ControllerTuningField::Stiffness(5)));
    EXPECT_FALSE(std::get<arm::JointImpedanceTuning>(presenter.FormTuning()).stiffness.has_value());
    view.ToggleFlag(ControllerTuningField::fromInertia, false);
    view.TypeNumber(ControllerTuningField::Stiffness(2), 150.0);
    const auto stiffness = std::get<arm::JointImpedanceTuning>(presenter.FormTuning()).stiffness;
    ASSERT_TRUE(stiffness.has_value());
    EXPECT_FLOAT_EQ(150.0f, (*stiffness)[2]);

    presenter.Choose(arm::ArmControllerKind::slotineLi);
    EXPECT_TRUE(view.IsControlVisible(ControllerTuningField::adaptationGain));
    EXPECT_TRUE(std::holds_alternative<arm::SlotineLiTuning>(presenter.FormTuning()));
    presenter.Choose(arm::ArmControllerKind::free);
    EXPECT_EQ(arm::ArmControllerKind::slotineLi, presenter.Kind());
}

namespace
{
    void AddPreset(arm_host::ArmSnapshot& snapshot, const char* name, arm::ArmControllerKind kind, const arm::ArmControllerTuning& tuning)
    {
        auto& preset = snapshot.config.presets[snapshot.config.presetCount++];
        preset.name = name;
        preset.controller = kind;
        preset.tuning = tuning;
    }
}

TEST_F(ArmControllerTuningPresenterTest, lists_the_chosen_controllers_presets_and_loads_one_by_applying_it)
{
    AddPreset(snapshot, "soft", arm::ArmControllerKind::gravityPd, arm::BandwidthTuning{ 10.0f, 1.0f });
    AddPreset(snapshot, "other", arm::ArmControllerKind::gravityPid, arm::BandwidthTuning{ 15.0f, 0.7f });
    AddPreset(snapshot, "stiff", arm::ArmControllerKind::gravityPd, arm::BandwidthTuning{ 60.0f, 0.7f });
    presenter.Refresh(snapshot, {});

    EXPECT_EQ((std::vector<std::string>{ "soft", "stiff" }), presenter.Presets());
    EXPECT_EQ("Choose a preset", presenter.LoadRefusal(2));
    EXPECT_EQ("", presenter.LoadRefusal(1));

    presenter.LoadPreset(1);
    EXPECT_EQ("Loaded “stiff”", presenter.PresetNote());
    ASSERT_EQ(1u, posted.size());
    EXPECT_FLOAT_EQ(60.0f, std::get<arm::BandwidthTuning>(Posted().tuning).bandwidth);
    EXPECT_EQ(ControllerTuningState::requested, presenter.State());
    EXPECT_EQ("Requested…", presenter.LoadRefusal(0));

    presenter.Choose(arm::ArmControllerKind::gravityPid);
    EXPECT_EQ((std::vector<std::string>{ "other" }), presenter.Presets());
}

TEST_F(ArmControllerTuningPresenterTest, loading_a_preset_is_refused_with_the_reason_while_moving)
{
    AddPreset(snapshot, "soft", arm::ArmControllerKind::gravityPd, arm::BandwidthTuning{ 10.0f, 1.0f });
    snapshot.moving = true;
    presenter.Refresh(snapshot, {});

    EXPECT_EQ("Moving: wait or abort", presenter.LoadRefusal(0));
    presenter.LoadPreset(0);
    EXPECT_TRUE(posted.empty());
    EXPECT_EQ("Load: Moving: wait or abort", presenter.PresetNote());
}

TEST_F(ArmControllerTuningPresenterTest, saves_the_applied_tuning_under_a_name_and_says_when_it_replaces_one)
{
    AddPreset(snapshot, "soft", arm::ArmControllerKind::gravityPd, arm::BandwidthTuning{ 10.0f, 1.0f });
    presenter.Refresh(snapshot, {});

    EXPECT_TRUE(presenter.Replaces("soft"));
    EXPECT_FALSE(presenter.Replaces("new"));
    EXPECT_EQ("Name it (1–16 characters)", presenter.SaveRefusal(""));
    EXPECT_EQ("Name it (1–16 characters)", presenter.SaveRefusal("seventeen chars!!"));

    presenter.SavePreset("new");
    ASSERT_EQ(1u, posted.size());
    const auto& save = std::get<arm_host::SaveArmControllerPreset>(posted.back());
    EXPECT_EQ(arm::ArmControllerKind::gravityPd, save.kind);
    EXPECT_EQ("new", save.name);

    EXPECT_EQ("Saved “new” with the applied tuning", presenter.PresetNote());
    presenter.SavePreset("soft");
    EXPECT_EQ("Replaced “soft” with the applied tuning", presenter.PresetNote());

    presenter.SavePreset("");
    EXPECT_EQ(2u, posted.size());
    EXPECT_EQ("Save: Name it (1–16 characters)", presenter.PresetNote());
}

TEST_F(ArmControllerTuningPresenterTest, a_seventeenth_preset_is_refused_unless_it_replaces_one)
{
    for (char name = 'a'; name != 'a' + static_cast<char>(arm::maxArmControllerPresets); ++name)
    {
        const char text[2]{ name, '\0' };
        AddPreset(snapshot, text, arm::ArmControllerKind::gravityPd, arm::BandwidthTuning{});
    }
    presenter.Refresh(snapshot, {});

    EXPECT_EQ("16 presets already: replace one", presenter.SaveRefusal("q"));
    EXPECT_EQ("", presenter.SaveRefusal("a"));
    auto disconnected = snapshot;
    disconnected.targetReady = false;
    presenter.Refresh(disconnected, {});
    EXPECT_EQ("Not connected", presenter.SaveRefusal("a"));
    EXPECT_EQ("Not connected", presenter.LoadRefusal(0));
}
