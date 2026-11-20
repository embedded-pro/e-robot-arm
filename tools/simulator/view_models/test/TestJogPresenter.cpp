#include "tools/simulator/view_models/JogPresenter.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <variant>
#include <vector>

namespace
{
    using namespace std::chrono_literals;
    using simulator::JogSpace;

    class JogPresenterTest
        : public testing::Test
    {
    public:
        static arm_host::ArmSnapshot Enabled()
        {
            auto snapshot = simulator::test::Running();
            snapshot.config.controlPeriod = 10ms;
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                snapshot.config.motionLimits[joint] = { foc::RadiansPerSecond{ 1.0f + static_cast<float>(joint) }, foc::RadiansPerSecondSquared{ 2.0f } };
            snapshot.config.toolLimits.speedMPerS = 0.25f;
            snapshot.config.toolLimits.angularSpeedRadPerS = 1.0f;
            return snapshot;
        }

        static arm_host::ArmSnapshot Moving()
        {
            auto snapshot = Enabled();
            snapshot.state = arm::ArmState::moving;
            snapshot.moving = true;
            return snapshot;
        }

        const arm_host::JogJoint& LastJointJog() const
        {
            return std::get<arm_host::JogJoint>(posted.back());
        }

        const arm::ToolJog& LastToolJog() const
        {
            return std::get<arm_host::JogTool>(posted.back()).jog;
        }

        simulator::JogPresenter::Clock::time_point start{};
        std::vector<arm_host::ArmCommand> posted;
        simulator::JogPresenter presenter{ [this](const arm_host::ArmCommand& command)
            {
                posted.push_back(command);
            } };
    };
}

TEST_F(JogPresenterTest, jogs_a_joint_at_a_share_of_its_speed_limit_while_held_and_stops_it_on_release)
{
    presenter.Refresh(Enabled(), start);
    ASSERT_TRUE(presenter.JogState().enabled);
    EXPECT_EQ("J2", presenter.Rows()[1].label);
    EXPECT_EQ("0.200 rad/s", presenter.Rows()[1].speed);

    presenter.Press(1, -1, start);
    ASSERT_EQ(1u, posted.size());
    EXPECT_EQ(1u, LastJointJog().joint);
    EXPECT_FLOAT_EQ(-0.2f, LastJointJog().speed.Value());
    EXPECT_TRUE(presenter.Held());
    EXPECT_EQ("Jogging J2 −0.200 rad/s", presenter.Status());
    EXPECT_FALSE(presenter.SpaceState().enabled);

    presenter.Release();
    ASSERT_EQ(2u, posted.size());
    EXPECT_EQ(1u, LastJointJog().joint);
    EXPECT_FLOAT_EQ(0.0f, LastJointJog().speed.Value());
    EXPECT_FALSE(presenter.Held());
    EXPECT_TRUE(presenter.Status().empty());
}

TEST_F(JogPresenterTest, sends_the_jog_again_every_half_of_the_arms_dead_man_time_while_held)
{
    presenter.Refresh(Enabled(), start);
    presenter.Press(0, 1, start);
    presenter.Refresh(Moving(), start + 90ms);
    EXPECT_EQ(1u, posted.size());

    presenter.Refresh(Moving(), start + 100ms);
    ASSERT_EQ(2u, posted.size());
    EXPECT_FLOAT_EQ(0.1f, LastJointJog().speed.Value());

    presenter.SetShare(50);
    presenter.Refresh(Moving(), start + 200ms);
    ASSERT_EQ(3u, posted.size());
    EXPECT_FLOAT_EQ(0.5f, LastJointJog().speed.Value());
}

TEST_F(JogPresenterTest, drops_a_held_jog_without_sending_when_the_arm_stops_being_enabled)
{
    presenter.Refresh(Enabled(), start);
    presenter.Press(0, 1, start);
    auto tripped = Moving();
    tripped.state = arm::ArmState::tripped;

    presenter.Refresh(tripped, start + 150ms);

    EXPECT_EQ(1u, posted.size());
    EXPECT_FALSE(presenter.Held());
    EXPECT_EQ("Tripped", presenter.JogState().reason);
    presenter.Release();
    EXPECT_EQ(1u, posted.size());
}

TEST_F(JogPresenterTest, jogs_the_tool_along_a_linear_or_rotational_axis_in_the_chosen_frame)
{
    presenter.Refresh(Enabled(), start);
    presenter.SetSpace(JogSpace::tool);
    presenter.SetShare(40);
    EXPECT_EQ("X", presenter.Rows()[0].label);
    EXPECT_EQ("0.100 m/s", presenter.Rows()[0].speed);
    EXPECT_EQ("Rz", presenter.Rows()[5].label);
    EXPECT_EQ("0.400 rad/s", presenter.Rows()[5].speed);

    presenter.Press(5, 1, start);
    EXPECT_EQ(arm::JogFrame::tool, LastToolJog().frame);
    EXPECT_EQ(arm::ToolAxis::rz, LastToolJog().axis);
    EXPECT_FLOAT_EQ(0.4f, LastToolJog().speed);

    presenter.SetSpace(JogSpace::base);
    EXPECT_EQ(JogSpace::tool, presenter.Space());
    presenter.Release();
    EXPECT_FLOAT_EQ(0.0f, LastToolJog().speed);

    presenter.SetSpace(JogSpace::base);
    presenter.Press(0, -1, start);
    EXPECT_EQ(arm::JogFrame::base, LastToolJog().frame);
    EXPECT_FLOAT_EQ(-0.1f, LastToolJog().speed);
}

TEST_F(JogPresenterTest, refuses_to_jog_unless_the_arm_is_enabled_and_says_why)
{
    presenter.Refresh(arm_host::ArmSnapshot{}, start);
    EXPECT_EQ("Not connected", presenter.JogState().reason);
    presenter.Refresh(simulator::test::Compatible(), start);
    EXPECT_EQ("Enable the arm first", presenter.JogState().reason);
    auto holding = Enabled();
    holding.state = arm::ArmState::holding;
    presenter.Refresh(holding, start);
    EXPECT_EQ("Holding: re-arm first", presenter.JogState().reason);
    presenter.Refresh(Moving(), start);
    EXPECT_EQ("Moving: wait or abort", presenter.JogState().reason);

    presenter.Press(0, 1, start);
    EXPECT_TRUE(posted.empty());
    EXPECT_FALSE(presenter.Held());
}

TEST_F(JogPresenterTest, keeps_the_share_between_one_and_a_hundred_percent_and_remembers_the_chosen_axis)
{
    presenter.Refresh(Enabled(), start);
    presenter.SetShare(0);
    EXPECT_EQ(1, presenter.Share());
    presenter.SetShare(250);
    EXPECT_EQ(100, presenter.Share());

    presenter.ChooseAxis(3);
    EXPECT_EQ(3u, presenter.ChosenAxis());
    presenter.ChooseAxis(9);
    EXPECT_EQ(3u, presenter.ChosenAxis());
    presenter.Press(4, 1, start);
    EXPECT_EQ(4u, presenter.ChosenAxis());
    presenter.Press(2, 1, start);
    EXPECT_EQ(1u, posted.size());
}
