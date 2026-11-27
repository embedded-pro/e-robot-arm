#include "tools/simulator/view_models/JointDrag.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <variant>
#include <vector>

namespace
{
    using namespace std::chrono_literals;
    using simulator::JointDrag;

    class JointDragTest
        : public testing::Test
    {
    public:
        JointDragTest()
        {
            snapshot = simulator::test::Running();
            snapshot.config.positionLimits = simulator::test::Limits();
            snapshot.disturbances = arm_host::Disturbances{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                snapshot.readings[joint].position = foc::Radians{ pose[joint] };
        }

        arm::JointPositions Turned(std::size_t joint, float by) const
        {
            arm::JointPositions joints{};
            for (std::size_t index = 0; index != arm::jointCount; ++index)
                joints[index] = foc::Radians{ pose[index] };
            joints[joint] = foc::Radians{ pose[joint] + by };
            return joints;
        }

        simulator::ScenePoint PointOn(std::size_t link, const arm::JointPositions& joints) const
        {
            return simulator::ToScenePoint(model.LinkPoses(joints)[link].Apply(offset));
        }

        static simulator::SceneRay FromAbove(const simulator::ScenePoint& point)
        {
            return { { point[0], point[1] + 300.0f, point[2] }, { 0.0f, -1.0f, 0.0f } };
        }

        bool GrabBase(bool follow = false)
        {
            return drag.Grab(0, PointOn(0, Turned(0, 0.0f)), snapshot, move, follow);
        }

        void DragBaseBy(float by, JointDrag::Clock::time_point at = {})
        {
            drag.Drag(FromAbove(PointOn(0, Turned(0, by))), snapshot, move, at);
        }

        template<class Command>
        bool Posted(std::size_t index) const
        {
            return index < posted.size() && std::holds_alternative<Command>(posted[index]);
        }

        std::array<float, arm::jointCount> pose{ 0.2f, 0.4f, 0.5f, 0.3f, 0.8f, -0.2f };
        arm_host::ArmSnapshot snapshot;
        simulator::ControlState move{ true, {} };
        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), simulator::test::Limits() };
        math::Vector3<float> offset{ 0.05f, 0.02f, 0.03f };
        std::vector<arm_host::ArmCommand> posted;
        JointDrag drag{ [this](const arm_host::ArmCommand& command)
            {
                posted.push_back(command);
            } };
    };
}

TEST_F(JointDragTest, grabs_only_a_link_on_a_connected_target_and_posts_nothing)
{
    const auto hit = PointOn(2, Turned(2, 0.0f));
    EXPECT_FALSE(drag.Grab(-1, hit, snapshot, move, false));
    EXPECT_FALSE(drag.Grab(6, hit, snapshot, move, false));
    auto disconnected = snapshot;
    disconnected.targetReady = false;
    EXPECT_FALSE(drag.Grab(2, hit, disconnected, move, false));
    EXPECT_FALSE(drag.Dragging());

    EXPECT_TRUE(drag.Grab(2, hit, snapshot, move, false));
    EXPECT_TRUE(drag.Dragging());
    EXPECT_FALSE(drag.Target().has_value());
    EXPECT_TRUE(posted.empty());
}

TEST_F(JointDragTest, the_target_turns_only_the_dragged_joint_to_follow_the_pointer)
{
    ASSERT_TRUE(GrabBase());
    DragBaseBy(0.3f);

    ASSERT_TRUE(drag.Target().has_value());
    const auto expected = Turned(0, 0.3f);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(expected[joint].Value(), (*drag.Target())[joint].Value(), 1.0e-3f) << "joint " << joint + 1;
    EXPECT_TRUE(posted.empty());

    DragBaseBy(-0.2f);
    EXPECT_NEAR(0.0f, (*drag.Target())[0].Value(), 1.0e-3f);
}

TEST_F(JointDragTest, a_joint_whose_axis_lies_across_the_view_turns_until_its_link_meets_the_pointer)
{
    const auto joints = Turned(2, 0.0f);
    ASSERT_TRUE(drag.Grab(2, PointOn(2, joints), snapshot, move, false));
    drag.Drag(FromAbove(PointOn(2, Turned(2, 0.2f))), snapshot, move, {});

    ASSERT_TRUE(drag.Target().has_value());
    EXPECT_NEAR(pose[2] + 0.2f, (*drag.Target())[2].Value(), 1.0e-3f);
    EXPECT_FLOAT_EQ(pose[1], (*drag.Target())[1].Value());
}

TEST_F(JointDragTest, the_target_stops_at_the_joint_limit_and_says_so)
{
    snapshot.config.positionLimits[0] = { foc::Radians{ -0.5f }, foc::Radians{ 0.4f } };
    ASSERT_TRUE(GrabBase());
    DragBaseBy(0.6f);

    EXPECT_FLOAT_EQ(0.4f, (*drag.Target())[0].Value());
    EXPECT_EQ("J1 → 0.40 rad (from 0.20) · at limit · Enter: move · Del: clear", drag.Label(move));
}

TEST_F(JointDragTest, enter_moves_to_the_target_once_the_drag_ends_and_move_is_enabled)
{
    EXPECT_FALSE(drag.Confirm(move));
    ASSERT_TRUE(GrabBase());
    DragBaseBy(0.3f);
    EXPECT_FALSE(drag.Confirm(move));

    drag.Release();
    EXPECT_FALSE(drag.Dragging());
    EXPECT_EQ("J1 → 0.50 rad (from 0.20) · Enter: move · Del: clear", drag.Label(move));
    const simulator::ControlState refused{ false, "the arm is moving" };
    EXPECT_EQ("J1 → 0.50 rad (from 0.20) · Move: the arm is moving", drag.Label(refused));
    EXPECT_FALSE(drag.Confirm(refused));
    EXPECT_TRUE(posted.empty());

    EXPECT_TRUE(drag.Confirm(move));
    ASSERT_TRUE(Posted<arm_host::MoveTo>(0));
    EXPECT_NEAR(0.5f, std::get<arm_host::MoveTo>(posted[0]).target[0].Value(), 1.0e-3f);
    EXPECT_FALSE(drag.Target().has_value());
    EXPECT_EQ("", drag.Label(move));
}

TEST_F(JointDragTest, escape_clears_the_target)
{
    ASSERT_TRUE(GrabBase());
    DragBaseBy(0.3f);
    drag.Release();
    drag.Cancel();

    EXPECT_FALSE(drag.Target().has_value());
    EXPECT_FALSE(drag.Confirm(move));
    EXPECT_TRUE(posted.empty());
}

TEST_F(JointDragTest, follow_moves_the_arm_at_most_twenty_times_a_second_and_aborts_on_release)
{
    ASSERT_TRUE(GrabBase(true));
    EXPECT_TRUE(drag.Following());
    const JointDrag::Clock::time_point start{};

    DragBaseBy(0.1f, start);
    DragBaseBy(0.2f, start + 20ms);
    DragBaseBy(0.3f, start + 50ms);
    ASSERT_EQ(2u, posted.size());
    EXPECT_TRUE(Posted<arm_host::MoveTo>(1));
    EXPECT_NEAR(0.5f, std::get<arm_host::MoveTo>(posted[1]).target[0].Value(), 1.0e-3f);
    EXPECT_EQ("J1 → 0.50 rad (from 0.20) · following", drag.Label(move));

    drag.Release();
    ASSERT_EQ(3u, posted.size());
    EXPECT_TRUE(Posted<arm_host::AbortMove>(2));
    EXPECT_FALSE(drag.Following());
    EXPECT_FALSE(drag.Target().has_value());
}

TEST_F(JointDragTest, follow_is_a_target_only_off_a_simulation)
{
    snapshot.disturbances.reset();
    ASSERT_TRUE(GrabBase(true));
    EXPECT_FALSE(drag.Following());
    DragBaseBy(0.3f);
    drag.Release();

    EXPECT_TRUE(posted.empty());
    EXPECT_EQ("J1 → 0.50 rad (from 0.20) · follow: simulation only · Enter: move · Del: clear", drag.Label(move));
}

TEST_F(JointDragTest, follow_stops_when_move_is_no_longer_allowed)
{
    ASSERT_TRUE(GrabBase(true));
    DragBaseBy(0.1f);
    move = { false, "the arm tripped" };
    DragBaseBy(0.2f);

    ASSERT_EQ(2u, posted.size());
    EXPECT_TRUE(Posted<arm_host::AbortMove>(1));
    EXPECT_FALSE(drag.Dragging());
    EXPECT_FALSE(drag.Target().has_value());
}

TEST_F(JointDragTest, a_new_grab_while_following_aborts_the_running_follow)
{
    ASSERT_TRUE(GrabBase(true));
    DragBaseBy(0.1f);
    ASSERT_TRUE(GrabBase(false));

    ASSERT_EQ(2u, posted.size());
    EXPECT_TRUE(Posted<arm_host::AbortMove>(1));
    EXPECT_FALSE(drag.Following());
}

TEST_F(JointDragTest, cancel_while_following_aborts_the_move_and_clears_the_target)
{
    ASSERT_TRUE(GrabBase(true));
    DragBaseBy(0.1f);
    drag.Cancel();

    ASSERT_EQ(2u, posted.size());
    EXPECT_TRUE(Posted<arm_host::AbortMove>(1));
    EXPECT_FALSE(drag.Target().has_value());
}

TEST_F(JointDragTest, the_ghost_shows_the_target_pose_and_the_label_sits_on_the_grabbed_point)
{
    EXPECT_FALSE(drag.TargetNodes().has_value());
    EXPECT_FALSE(drag.TargetPoint().has_value());
    ASSERT_TRUE(GrabBase());
    DragBaseBy(0.3f);

    const auto expected = simulator::SceneNodePoses(model, snapshot.config.geometry, Turned(0, 0.3f));
    ASSERT_TRUE(drag.TargetNodes().has_value());
    for (std::size_t node = 0; node != simulator::sceneNodeCount; ++node)
        for (std::size_t axis = 0; axis != 3; ++axis)
            EXPECT_NEAR(expected[node].position[axis], (*drag.TargetNodes())[node].position[axis], 0.05f) << "node " << node;
    const auto point = PointOn(0, Turned(0, 0.3f));
    ASSERT_TRUE(drag.TargetPoint().has_value());
    for (std::size_t axis = 0; axis != 3; ++axis)
        EXPECT_NEAR(point[axis], (*drag.TargetPoint())[axis], 0.05f);
}

TEST_F(JointDragTest, a_point_a_millimetre_from_the_joint_axis_leaves_the_joint_where_it_is)
{
    auto elbow = simulator::ToScenePoint(model.LinkPoses(Turned(2, 0.0f))[2].Apply(math::Vector3<float>{ 0.0f, 0.0f, 0.0f }));
    elbow[1] += 0.1f;
    ASSERT_TRUE(drag.Grab(2, elbow, snapshot, move, false));
    drag.Drag({ { elbow[0], elbow[1] + 3.0f, elbow[2] + 300.0f }, { 0.0f, 0.0f, -1.0f } }, snapshot, move, {});

    ASSERT_TRUE(drag.Target().has_value());
    EXPECT_FLOAT_EQ(pose[2], (*drag.Target())[2].Value());
}
