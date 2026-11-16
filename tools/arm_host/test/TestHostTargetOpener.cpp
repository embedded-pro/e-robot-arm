#include "infra/timer/test_helper/ClockFixture.hpp"
#include "tools/arm_host/AdapterTargetLink.hpp"
#include "tools/arm_host/test/ArmHostTestSupport.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    using namespace std::chrono_literals;

    class HostTargetOpenerTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        arm_host::TargetOpener opener{ arm_host::HostTargetOpener(50ms, 1000000) };
    };
}

TEST_F(HostTargetOpenerTest, opens_simulated_joints_in_the_process)
{
    const auto link = opener({ arm_host::TargetKind::simulated, "" }, arm_host::test::Config());

    ASSERT_NE(nullptr, link);
    EXPECT_NE(nullptr, link->Simulation());
    EXPECT_EQ(infra::Duration{ 2s }, link->ConnectTimeout());
    EXPECT_EQ(std::nullopt, link->Failure());
    EXPECT_EQ(nullptr, link->Motors());
    EXPECT_FALSE(link->Adopt(arm_host::test::Config()));
}

TEST_F(HostTargetOpenerTest, leaves_the_virtual_arm_to_its_own_opener)
{
    EXPECT_EQ(nullptr, opener({ arm_host::TargetKind::virtualArm, "virtual:arm", "e-foc-virtual-motor" }, arm_host::test::Config()));
}

TEST_F(HostTargetOpenerTest, an_adapter_that_cannot_be_opened_gives_no_link)
{
    EXPECT_EQ(nullptr, opener({ arm_host::TargetKind::real, "socketcan:no-such-interface" }, arm_host::test::Config()));
}
