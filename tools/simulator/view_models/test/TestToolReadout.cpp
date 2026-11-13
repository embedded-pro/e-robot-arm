#include "tools/simulator/view_models/ToolReadout.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"

namespace
{
    class ToolReadoutTest
        : public testing::Test
    {
    public:
        arm_host::ArmSnapshot Live() const
        {
            auto snapshot = simulator::test::Running();
            snapshot.toolPose = { 0.58f, -0.0001f, 0.15f, 0.0f, 1.5708f, -0.25f };
            snapshot.manipulability = 0.001234f;
            return snapshot;
        }
    };
}

TEST_F(ToolReadoutTest, formats_the_tool_pose_and_manipulability_with_units)
{
    const auto readout = simulator::ToolReadoutOf(Live());

    EXPECT_EQ("x 0.580  y 0.000  z 0.150 m", readout.position);
    EXPECT_EQ("roll 0.000  pitch 1.571  yaw -0.250 rad", readout.rotation);
    EXPECT_EQ("1.23e-03", readout.manipulability);
    EXPECT_EQ("—", readout.singularity);
    EXPECT_EQ(ui::theme::StatusLevel::Neutral, readout.singularityLevel);
    EXPECT_TRUE(readout.warning.empty());
}

TEST_F(ToolReadoutTest, names_the_singularities_the_arm_is_near_as_a_warning)
{
    auto snapshot = Live();
    snapshot.singularities = { false, true, false };
    const auto one = simulator::ToolReadoutOf(snapshot);
    EXPECT_EQ("elbow", one.singularity);
    EXPECT_EQ(ui::theme::StatusLevel::Warning, one.singularityLevel);
    EXPECT_EQ("Near elbow singularity", one.warning);

    snapshot.singularities = { true, false, true };
    EXPECT_EQ("Near wrist, shoulder singularities", simulator::ToolReadoutOf(snapshot).warning);
}

TEST_F(ToolReadoutTest, shows_placeholders_and_no_warning_without_a_live_arm)
{
    auto snapshot = Live();
    snapshot.singularities = { true, true, false };
    snapshot.readings[4].silent = true;
    const auto silent = simulator::ToolReadoutOf(snapshot);
    EXPECT_EQ("—", silent.position);
    EXPECT_EQ("—", silent.manipulability);
    EXPECT_TRUE(silent.warning.empty());

    EXPECT_EQ(silent, simulator::ToolReadoutOf(arm_host::ArmSnapshot{}));
}
