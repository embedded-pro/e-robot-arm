#include "tools/simulator/view_models/HandGuide.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"
#include <vector>

namespace
{
    constexpr float centimetres{ 100.0f };

    math::Vector3<float> ToBase(const simulator::ScenePoint& point)
    {
        return math::Vector3<float>{ point[0] / centimetres, -point[2] / centimetres, point[1] / centimetres };
    }

    simulator::ScenePoint ToScene(const math::Vector3<float>& point)
    {
        return { point[0] * centimetres, point[2] * centimetres, -point[1] * centimetres };
    }

    class HandGuideTest
        : public testing::Test
    {
    public:
        HandGuideTest()
        {
            snapshot = simulator::test::Running();
            snapshot.config.positionLimits = simulator::test::Limits();
            snapshot.disturbances = arm_host::Disturbances{ .collision = true };
            const std::array<float, arm::jointCount> pose{ 0.2f, 0.4f, 0.5f, 0.3f, 0.8f, -0.2f };
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                snapshot.readings[joint].position = foc::Radians{ pose[joint] };
        }

        arm::JointPositions Joints() const
        {
            arm::JointPositions joints{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                joints[joint] = snapshot.readings[joint].position;
            return joints;
        }

        math::Vector3<float> PointOn(std::size_t link, const arm::JointPositions& joints) const
        {
            return model.LinkPoses(joints)[link].Apply(offset);
        }

        simulator::SceneRay RayThrough(const simulator::ScenePoint& point) const
        {
            return { { point[0], point[1], point[2] + 300.0f }, { 0.0f, 0.0f, -1.0f } };
        }

        const arm::JointTorques& Posted() const
        {
            return std::get<arm_host::GuideArm>(posted.back()).torques;
        }

        arm_host::ArmSnapshot snapshot;
        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), simulator::test::Limits() };
        math::Vector3<float> offset{ 0.02f, 0.01f, 0.03f };
        std::vector<arm_host::ArmCommand> posted;
        simulator::HandGuide guide{ [this](const arm_host::ArmCommand& command)
            {
                posted.push_back(command);
            } };
    };
}

TEST_F(HandGuideTest, grabs_only_a_link_and_only_where_the_target_takes_a_torque)
{
    const auto hit = ToScene(PointOn(2, Joints()));

    EXPECT_FALSE(guide.Grab(-1, hit, snapshot));
    EXPECT_FALSE(guide.Grab(6, hit, snapshot));
    auto real = snapshot;
    real.disturbances.reset();
    EXPECT_FALSE(guide.Grab(2, hit, real));
    EXPECT_FALSE(guide.Active());

    EXPECT_TRUE(guide.Grab(2, hit, snapshot));
    EXPECT_TRUE(guide.Active());
    EXPECT_TRUE(posted.empty());
}

TEST_F(HandGuideTest, a_pointer_on_the_grabbed_point_pulls_with_no_force)
{
    const auto hit = ToScene(PointOn(2, Joints()));
    ASSERT_TRUE(guide.Grab(2, hit, snapshot));

    guide.Drag(RayThrough(hit), snapshot);

    ASSERT_EQ(1u, posted.size());
    for (const auto torque : Posted())
        EXPECT_NEAR(0.0f, torque.Value(), 1.0e-3f);
    EXPECT_NEAR(0.0f, guide.Force(), 1.0e-3f);
}

TEST_F(HandGuideTest, the_torques_do_the_springs_work_on_the_grabbed_point_and_spare_the_joints_beyond_its_link)
{
    const auto joints = Joints();
    const auto point = PointOn(2, joints);
    ASSERT_TRUE(guide.Grab(2, ToScene(point), snapshot));
    auto pointer = ToScene(point);
    pointer[0] += 5.0f;
    pointer[1] += 3.0f;

    guide.Drag(RayThrough(pointer), snapshot);

    const auto& torques = Posted();
    ASSERT_TRUE(guide.Arrow().has_value());
    const auto pull = (ToBase(guide.Arrow()->to) - ToBase(guide.Arrow()->from)) * simulator::HandGuide::stiffness;
    EXPECT_NEAR(std::sqrt(pull[0] * pull[0] + pull[1] * pull[1] + pull[2] * pull[2]), guide.Force(), 1.0e-3f);
    EXPECT_NEAR(23.3f, guide.Force(), 0.1f);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        auto moved = joints;
        moved[joint] = foc::Radians{ moved[joint].Value() + 1.0e-3f };
        const auto shift = PointOn(2, moved) - point;
        const auto work = shift[0] * pull[0] + shift[1] * pull[1] + shift[2] * pull[2];
        EXPECT_NEAR(work, torques[joint].Value() * 1.0e-3f, 2.0e-4f) << "joint " << joint + 1;
    }
    EXPECT_FLOAT_EQ(0.0f, torques[3].Value());
    EXPECT_FLOAT_EQ(0.0f, torques[5].Value());
}

TEST_F(HandGuideTest, caps_the_pull_at_100_n_and_names_the_joint_in_the_label)
{
    const auto point = ToScene(PointOn(2, Joints()));
    ASSERT_TRUE(guide.Grab(2, point, snapshot));
    auto pointer = point;
    pointer[0] += 100.0f;

    guide.Drag(RayThrough(pointer), snapshot);

    EXPECT_FLOAT_EQ(simulator::HandGuide::maximumForce, guide.Force());
    EXPECT_EQ("J3 · guided · 100 N", guide.Label());
    for (const auto torque : Posted())
        EXPECT_LE(std::abs(torque.Value()), simulator::HandGuide::maximumTorque);
}

TEST_F(HandGuideTest, release_sends_no_torque_once_and_clears_the_arrow)
{
    const auto point = ToScene(PointOn(1, Joints()));
    ASSERT_TRUE(guide.Grab(1, point, snapshot));
    guide.Drag(RayThrough(point), snapshot);

    guide.Release();
    guide.Release();

    ASSERT_EQ(2u, posted.size());
    EXPECT_EQ(arm::JointTorques{}, Posted());
    EXPECT_FALSE(guide.Active());
    EXPECT_FALSE(guide.Arrow().has_value());
    EXPECT_EQ("", guide.Label());
}

TEST_F(HandGuideTest, a_target_that_stops_offering_torque_ends_the_drag)
{
    const auto point = ToScene(PointOn(1, Joints()));
    ASSERT_TRUE(guide.Grab(1, point, snapshot));
    auto disconnected = snapshot;
    disconnected.targetReady = false;

    guide.Drag(RayThrough(point), disconnected);

    ASSERT_EQ(1u, posted.size());
    EXPECT_EQ(arm::JointTorques{}, Posted());
    EXPECT_FALSE(guide.Active());
}
