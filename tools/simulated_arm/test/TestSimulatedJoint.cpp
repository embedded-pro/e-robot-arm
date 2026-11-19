#include "e-foc-protocol/client/FocMotorCanClient.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "tools/simulated_arm/SimulatedArm.hpp"
#include "gtest/gtest.h"

namespace
{
    using namespace std::chrono_literals;

    class SimulatedJointTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        simulation::SimulatedArm arm{ arm::JointNodeIds{ 1, 2, 3, 4, 5, 6 }, 0ms };
        can::FocMotorCanClient client{ arm.ArmSide() };
    };
}

TEST_F(SimulatedJointTest, a_quick_stop_holds_a_running_joint_and_reports_it_ready)
{
    client.CategoryClient().SendQueryContractVersion(1);
    ForwardTime(10ms);
    client.CategoryClient().SendStart(1);
    ForwardTime(10ms);
    ASSERT_EQ(can::FocMotorState::running, arm.Joint(0).State());

    client.CategoryClient().SendQuickStop(1);
    ForwardTime(10ms);

    EXPECT_EQ(can::FocMotorState::ready, arm.Joint(0).State());
}

TEST_F(SimulatedJointTest, a_silenced_joint_ignores_the_bus_until_it_is_resumed)
{
    arm.Joint(1).Silence();
    EXPECT_TRUE(arm.Joint(1).Silent());
    client.CategoryClient().SendEmergencyStop(2);
    ForwardTime(10ms);
    EXPECT_EQ(0u, arm.Joint(1).EmergencyStops());

    arm.Joint(1).Resume();
    EXPECT_FALSE(arm.Joint(1).Silent());
    client.CategoryClient().SendEmergencyStop(2);
    ForwardTime(10ms);
    EXPECT_EQ(1u, arm.Joint(1).EmergencyStops());
}

TEST_F(SimulatedJointTest, a_running_joint_with_a_command_timeout_faults_once_its_setpoints_stop)
{
    client.CategoryClient().SendQueryContractVersion(1);
    ForwardTime(10ms);
    client.CategoryClient().SendSetParameter(1, can::FocParameter::commandTimeout, 0.05f);
    client.CategoryClient().SendStart(1);
    ForwardTime(10ms);
    ASSERT_EQ(can::FocMotorState::running, arm.Joint(0).State());

    for (int period = 0; period != 10; ++period)
    {
        client.CategoryClient().SendSetpoint(1, can::FocSetpoint{ can::FocSetpointKind::torqueNewtonMeter, 0.0f });
        ForwardTime(10ms);
    }
    EXPECT_EQ(can::FocMotorState::running, arm.Joint(0).State());

    ForwardTime(60ms);
    EXPECT_EQ(can::FocMotorState::fault, arm.Joint(0).State());
}
