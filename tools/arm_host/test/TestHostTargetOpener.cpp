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
        arm_host::TargetOpener opener{ arm_host::HostTargetOpener(arm_host::test::nodes, 50ms, 1000000) };
    };
}

TEST_F(HostTargetOpenerTest, opens_simulated_joints_in_the_process)
{
    const auto link = opener({ arm_host::TargetKind::simulated, "" });

    ASSERT_NE(nullptr, link);
    EXPECT_NE(nullptr, link->Simulation());
}

TEST_F(HostTargetOpenerTest, an_adapter_that_cannot_be_opened_gives_no_link)
{
    EXPECT_EQ(nullptr, opener({ arm_host::TargetKind::real, "socketcan:no-such-interface" }));
}
