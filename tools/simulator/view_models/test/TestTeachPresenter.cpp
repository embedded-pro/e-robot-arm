#include "tools/simulator/view_models/TeachPresenter.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"
#include <string>
#include <variant>
#include <vector>

namespace
{
    class TeachPresenterTest
        : public testing::Test
    {
    public:
        static arm_host::ArmSnapshot WithPoints(arm_host::ArmSnapshot snapshot, std::size_t count = 2)
        {
            snapshot.config.teachPointCount = static_cast<uint8_t>(count);
            for (std::size_t point = 0; point != count; ++point)
            {
                auto& taught = snapshot.config.teachPoints[point];
                const auto name = point == 0 ? std::string{ "home" } : point == 1 ? std::string{ "pick" }
                                                                                  : "p" + std::to_string(point);
                taught.name.assign(name.data(), name.size());
                for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                    taught.joints[joint] = foc::Radians{ 0.1f * static_cast<float>(point * joint) };
            }
            return snapshot;
        }

        std::vector<arm_host::ArmCommand> posted;
        simulator::TeachPresenter presenter{ [this](const arm_host::ArmCommand& command)
            {
                posted.push_back(command);
            } };
    };
}

TEST_F(TeachPresenterTest, lists_the_profiles_teach_points_with_their_joint_positions)
{
    presenter.Refresh(WithPoints(simulator::test::Running()));

    ASSERT_EQ(2u, presenter.Rows().size());
    EXPECT_EQ("home", presenter.Rows()[0].name);
    EXPECT_EQ("pick", presenter.Rows()[1].name);
    EXPECT_EQ("0.300", presenter.Rows()[1].positions[3]);
}

TEST_F(TeachPresenterTest, teaches_the_current_pose_under_a_name_and_says_when_it_replaces_a_point)
{
    presenter.Refresh(WithPoints(simulator::test::Running()));
    EXPECT_EQ("Name it (1–16 characters)", presenter.TeachState().reason);

    presenter.SetName("place");
    ASSERT_TRUE(presenter.TeachState().enabled);
    EXPECT_TRUE(presenter.Hint().empty());
    presenter.Teach();
    ASSERT_EQ(1u, posted.size());
    EXPECT_EQ("place", std::get<arm_host::Teach>(posted.back()).name);

    presenter.Choose(1);
    EXPECT_EQ("pick", presenter.Name());
    EXPECT_EQ("Teach replaces \"pick\"", presenter.Hint());

    presenter.SetName("a name longer than sixteen");
    EXPECT_FALSE(presenter.TeachState().enabled);
    presenter.Teach();
    EXPECT_EQ(1u, posted.size());
}

TEST_F(TeachPresenterTest, refuses_to_teach_while_moving_or_with_sixteen_points_unless_it_replaces_one)
{
    auto moving = WithPoints(simulator::test::Running());
    moving.state = arm::ArmState::moving;
    moving.moving = true;
    presenter.Refresh(moving);
    presenter.SetName("place");
    EXPECT_EQ("Moving: wait or abort", presenter.TeachState().reason);

    presenter.Refresh(WithPoints(simulator::test::Running(), 16));
    EXPECT_EQ("16 points already: replace one", presenter.TeachState().reason);
    presenter.SetName("pick");
    EXPECT_TRUE(presenter.TeachState().enabled);

    presenter.Refresh(arm_host::ArmSnapshot{});
    EXPECT_EQ("Not connected", presenter.TeachState().reason);
}

TEST_F(TeachPresenterTest, goes_to_the_chosen_point_when_the_arm_may_move)
{
    presenter.Refresh(WithPoints(simulator::test::Compatible()));
    EXPECT_EQ("Choose a point", presenter.GoToState().reason);
    presenter.Choose(1);
    EXPECT_EQ("Enable the arm first", presenter.GoToState().reason);
    presenter.GoTo();
    EXPECT_TRUE(posted.empty());

    presenter.Refresh(WithPoints(simulator::test::Running()));
    ASSERT_TRUE(presenter.GoToState().enabled);
    presenter.GoTo();
    ASSERT_EQ(1u, posted.size());
    EXPECT_EQ("pick", std::get<arm_host::GoTo>(posted.back()).name);

    presenter.Refresh(WithPoints(simulator::test::Running(), 1));
    EXPECT_FALSE(presenter.Chosen().has_value());
}
