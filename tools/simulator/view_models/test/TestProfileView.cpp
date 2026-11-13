#include "tools/simulator/view_models/ProfileView.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"

namespace
{
    class ProfileViewTest
        : public testing::Test
    {
    public:
        ProfileViewTest()
        {
            snapshot.profileName = "bench arm";
            snapshot.config.controlPeriod = std::chrono::milliseconds{ 5 };
            snapshot.config.missedRequestsBeforeSilent = 4;
            snapshot.config.nodes = { 11, 12, 13, 14, 15, 16 };
            snapshot.config.transmissions[0] = { 50.0f, arm::JointDirection::negative, foc::Radians{ 0.125f } };
            snapshot.config.positionLimits[0] = { foc::Radians{ -1.5f }, foc::Radians{ 2.25f } };
            snapshot.config.motionLimits[0] = { foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecondSquared{ 2.5f } };
            arm::ControllerSettings settings{};
            settings.current = can::FocCurrentAlgorithm::slidingMode;
            settings.speed = can::FocSpeedAlgorithm::lqi;
            settings.position = can::FocPositionAlgorithm::lqr;
            snapshot.config.controllers[0] = settings;
            snapshot.config.motors[0] = arm::JointMotor{ 1, arm::motorPresets[1].settings };
            snapshot.config.motors[2] = arm::JointMotor{ 0, arm::motorPresets[1].settings };
        }

        arm_host::ArmSnapshot snapshot{ simulator::test::Connected() };
    };
}

TEST_F(ProfileViewTest, describes_the_profile_in_effect_joint_by_joint)
{
    const auto view = simulator::DescribeProfile(snapshot);

    EXPECT_EQ("bench arm", view.name);
    EXPECT_EQ("Control period 5 ms · Enable mode position · Silent after 4 · Command timeout 3", view.summary);
    EXPECT_EQ((simulator::ProfileRow{ "J1", "11", "50.00", "−", "0.125", "-1.500", "2.250", "1.000", "2.500", "sliding mode / LQI / LQR", "Anaheim BLY172S-24V-4000" }), view.rows[0]);
    EXPECT_EQ("node's own", view.rows[1].controller);
    EXPECT_EQ("node's own", view.rows[1].motor);
    EXPECT_EQ("custom", view.rows[2].motor);
    EXPECT_EQ("J6", view.rows[5].joint);
}

TEST_F(ProfileViewTest, shows_nothing_without_a_target)
{
    snapshot.targetReady = false;

    const auto view = simulator::DescribeProfile(snapshot);
    EXPECT_EQ("—", view.name);
    EXPECT_EQ("No target", view.summary);
}
