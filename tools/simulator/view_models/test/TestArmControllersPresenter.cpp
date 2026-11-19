#include "tools/simulator/view_models/ArmControllersPresenter.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"
#include <string>
#include <variant>
#include <vector>

namespace
{
    using simulator::ArmControllerAction;

    class ArmControllersPresenterTest
        : public testing::Test
    {
    public:
        static arm_host::ArmSnapshot WithControllers(arm_host::ArmSnapshot snapshot)
        {
            snapshot.armControllers = { { "gravity-pd", true, true }, { "computed-torque", true, false }, { "gravity-pid", false, false } };
            snapshot.config.gravityPd = { 30.0f, 0.7f };
            snapshot.config.computedTorque = { 25.0f, 0.8f };
            snapshot.config.gravityPid = { 20.0f, 0.7f };
            return snapshot;
        }

        std::vector<arm_host::ArmCommand> posted;
        std::string prompt;
        simulator::ArmControllersPresenter presenter{ [this](const arm_host::ArmCommand& command)
            {
                posted.push_back(command);
            } };
    };
}

TEST_F(ArmControllersPresenterTest, lists_the_registry_with_its_state_and_the_profiles_parameters)
{
    presenter.Refresh(WithControllers(simulator::test::Compatible()));

    const auto& rows = presenter.Rows();
    ASSERT_EQ(3u, rows.size());
    EXPECT_EQ("gravity-pd", rows[0].name);
    EXPECT_EQ("Active", rows[0].state);
    EXPECT_EQ(ui::theme::StatusLevel::Ok, rows[0].level);
    EXPECT_EQ("ω 30.0 rad/s, ζ 0.70", rows[0].parameters);
    EXPECT_EQ("Enabled", rows[1].state);
    EXPECT_EQ("ω 25.0 rad/s, ζ 0.80", rows[1].parameters);
    EXPECT_EQ("Disabled", rows[2].state);
    EXPECT_EQ(ui::theme::StatusLevel::Neutral, rows[2].level);
}

TEST_F(ArmControllersPresenterTest, without_a_chosen_row_every_action_says_to_choose_one)
{
    presenter.Refresh(WithControllers(simulator::test::Compatible()));

    for (const auto action : { ArmControllerAction::select, ArmControllerAction::enable, ArmControllerAction::disable })
    {
        EXPECT_FALSE(presenter.State(action).enabled);
        EXPECT_EQ("Choose a controller", presenter.State(action).reason);
    }
    presenter.Trigger(ArmControllerAction::select);
    EXPECT_TRUE(posted.empty());
}

TEST_F(ArmControllersPresenterTest, selects_an_enabled_controller_and_refuses_a_disabled_or_active_one)
{
    presenter.Refresh(WithControllers(simulator::test::Compatible()));

    presenter.Choose(1);
    EXPECT_TRUE(presenter.State(ArmControllerAction::select).enabled);
    EXPECT_EQ("Already enabled", presenter.State(ArmControllerAction::enable).reason);
    presenter.Trigger(ArmControllerAction::select);
    ASSERT_EQ(1u, posted.size());
    EXPECT_EQ(1u, std::get<arm_host::SelectArmController>(posted[0]).id);

    presenter.Choose(2);
    EXPECT_EQ("Enable it first", presenter.State(ArmControllerAction::select).reason);
    EXPECT_EQ("Already disabled", presenter.State(ArmControllerAction::disable).reason);
    presenter.Trigger(ArmControllerAction::enable);
    ASSERT_EQ(2u, posted.size());
    EXPECT_EQ(2u, std::get<arm_host::EnableArmController>(posted[1]).id);

    presenter.Choose(0);
    EXPECT_EQ("Already active", presenter.State(ArmControllerAction::select).reason);
}

TEST_F(ArmControllersPresenterTest, the_active_controller_is_not_disabled_while_the_arm_drives_its_joints)
{
    presenter.Refresh(WithControllers(simulator::test::Running()));
    presenter.Choose(0);

    EXPECT_FALSE(presenter.State(ArmControllerAction::disable).enabled);
    EXPECT_EQ("Disable the arm first", presenter.State(ArmControllerAction::disable).reason);
    EXPECT_EQ("Disable: disable the arm first", presenter.Hint());
    presenter.Trigger(ArmControllerAction::disable);
    EXPECT_TRUE(posted.empty());
}

TEST_F(ArmControllersPresenterTest, disabling_the_active_controller_asks_first_and_posts_only_once_confirmed)
{
    presenter.SetDisablePrompt([this](std::string_view text)
        {
            prompt = text;
        });
    presenter.Refresh(WithControllers(simulator::test::Compatible()));
    presenter.Choose(0);

    presenter.Trigger(ArmControllerAction::disable);
    EXPECT_EQ("Disable gravity-pd? The arm then has no controller and cannot be enabled until another one is selected.", prompt);
    EXPECT_TRUE(posted.empty());

    presenter.ConfirmDisable();
    ASSERT_EQ(1u, posted.size());
    EXPECT_EQ(0u, std::get<arm_host::DisableArmController>(posted[0]).id);
}

TEST_F(ArmControllersPresenterTest, disabling_an_inactive_controller_posts_at_once)
{
    presenter.Refresh(WithControllers(simulator::test::Running()));
    presenter.Choose(1);

    presenter.Trigger(ArmControllerAction::disable);

    ASSERT_EQ(1u, posted.size());
    EXPECT_EQ(1u, std::get<arm_host::DisableArmController>(posted[0]).id);
}

TEST_F(ArmControllersPresenterTest, a_row_that_disappears_with_a_new_profile_is_no_longer_chosen)
{
    presenter.Refresh(WithControllers(simulator::test::Compatible()));
    presenter.Choose(2);

    auto fewer = WithControllers(simulator::test::Compatible());
    fewer.armControllers.resize(1);
    presenter.Refresh(fewer);

    EXPECT_FALSE(presenter.Chosen().has_value());
    EXPECT_EQ("Choose a controller", presenter.State(ArmControllerAction::enable).reason);
}
