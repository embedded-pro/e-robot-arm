#include "core/workspace/WorkspaceCheck.hpp"
#include "gmock/gmock.h"

namespace
{
    arm::JointLimits WideLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.5f }, foc::Radians{ 3.5f } };
        return limits;
    }

    arm::JointPositions Pose(float j1, float j2, float j3, float j4 = 0.0f, float j5 = 0.0f, float j6 = 0.0f)
    {
        return { foc::Radians{ j1 }, foc::Radians{ j2 }, foc::Radians{ j3 }, foc::Radians{ j4 }, foc::Radians{ j5 }, foc::Radians{ j6 } };
    }

    class WorkspaceCheckTest
        : public testing::Test
    {
    public:
        arm::WorkspaceCheck Check(const arm::WorkspaceConfig& config = {}) const
        {
            return arm::WorkspaceCheck{ model, arm::ReferenceGeometry(), config };
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
    };
}

TEST_F(WorkspaceCheckTest, the_reference_arm_upright_is_inside_its_workspace_and_clear_of_itself)
{
    EXPECT_EQ(std::nullopt, Check().Check(Pose(0.0f, 0.0f, 0.0f)));
    EXPECT_EQ(std::nullopt, Check().Check(Pose(0.5f, 0.4f, 0.6f, 0.3f, 0.8f, 0.2f)));
}

TEST_F(WorkspaceCheckTest, the_link_capsules_chain_from_the_base_to_the_tool_with_the_profiles_radii)
{
    arm::WorkspaceConfig config{};
    config.linkRadii = { 0.07f, 0.06f, 0.05f, 0.04f, 0.03f, 0.02f, 0.01f };
    const auto joints = Pose(0.5f, 0.4f, 0.6f, 0.3f, 0.8f, 0.2f);

    const auto capsules = Check(config).LinkCapsules(joints);
    const auto tool = model.ToolPose(joints);

    for (std::size_t link = 0; link != arm::workspaceCapsules; ++link)
        EXPECT_FLOAT_EQ(config.linkRadii[link], capsules[link].radius) << "link " << link;
    for (std::size_t link = 0; link + 1 != arm::workspaceCapsules; ++link)
        for (std::size_t axis = 0; axis != 3; ++axis)
            EXPECT_FLOAT_EQ(capsules[link].axis.end[axis], capsules[link + 1].axis.start[axis]) << "link " << link << " axis " << axis;
    for (std::size_t axis = 0; axis != 3; ++axis)
        EXPECT_NEAR(tool.translation[axis], capsules.back().axis.end[axis], 1e-5f) << "axis " << axis;
}

TEST_F(WorkspaceCheckTest, the_default_workspace_spans_the_arms_reach_above_the_base_plane)
{
    const auto check = Check();
    const float reach{ 0.02f + 0.15f + 0.25f + 0.25f + 0.06f + 0.04f };

    EXPECT_NEAR(-reach, check.Workspace().min[0], 1e-5f);
    EXPECT_NEAR(reach, check.Workspace().max[1], 1e-5f);
    EXPECT_NEAR(0.0f, check.Workspace().min[2], 1e-6f);
    EXPECT_NEAR(reach, check.Workspace().max[2], 1e-5f);
}

TEST_F(WorkspaceCheckTest, a_link_reaching_below_the_base_plane_leaves_the_workspace)
{
    const auto violation = Check().Check(Pose(0.0f, 2.0f, 0.0f));

    ASSERT_TRUE(violation.has_value());
    EXPECT_EQ(arm::WorkspaceRule::workspace, violation->rule);
    EXPECT_GE(violation->link, 1);
    EXPECT_GT(violation->distance, 0.0f);
}

TEST_F(WorkspaceCheckTest, a_link_touching_a_forbidden_zone_names_the_link_and_the_zone)
{
    arm::WorkspaceConfig config{};
    config.forbiddenZones[1] = { math::Vector3<float>{ -0.1f, -0.1f, 0.68f }, math::Vector3<float>{ 0.1f, 0.1f, 0.8f } };
    config.forbiddenZoneCount = 2;
    config.forbiddenZones[0] = { math::Vector3<float>{ 1.0f, 1.0f, 0.0f }, math::Vector3<float>{ 1.2f, 1.2f, 0.2f } };

    const auto violation = Check(config).Check(Pose(0.0f, 0.0f, 0.0f));

    ASSERT_TRUE(violation.has_value());
    EXPECT_EQ(arm::WorkspaceRule::forbiddenZone, violation->rule);
    EXPECT_EQ(1, violation->other);
    EXPECT_GT(violation->distance, 0.0f);
}

TEST_F(WorkspaceCheckTest, a_forearm_folded_onto_the_column_collides_with_the_arm_itself)
{
    const auto violation = Check().Check(Pose(0.0f, 0.0f, 3.0f));

    ASSERT_TRUE(violation.has_value());
    EXPECT_EQ(arm::WorkspaceRule::selfCollision, violation->rule);
    EXPECT_LT(violation->distance, 0.01f);
}

TEST_F(WorkspaceCheckTest, a_pair_the_profile_allows_is_not_checked)
{
    const auto folded = Pose(0.0f, 0.0f, 3.0f);
    const auto first = Check().Check(folded);
    ASSERT_TRUE(first.has_value());

    arm::WorkspaceConfig config{};
    config.allowedLinkPairs[0] = { first->link, first->other };
    config.allowedLinkPairCount = 1;
    const auto second = Check(config).Check(folded);

    EXPECT_TRUE(!second.has_value() || !(second->link == first->link && second->other == first->other));
}
