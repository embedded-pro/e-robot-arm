#include "tools/arm_host/ArmHost.hpp"
#include "gmock/gmock.h"

namespace
{
    class ArmRequestQueueTest
        : public testing::Test
    {
    public:
        uint32_t PopId()
        {
            const auto request = queue.Pop();
            return request ? request->id : 0;
        }

        arm_host::ArmRequestQueue queue;
    };
}

TEST_F(ArmRequestQueueTest, hands_out_requests_in_order)
{
    queue.Push({ 1, arm_host::ArmCommand{ arm_host::StartJoints{} } });
    queue.Push({ 2, arm_host::ArmTarget{} });

    EXPECT_EQ(1u, PopId());
    EXPECT_EQ(2u, PopId());
    EXPECT_FALSE(queue.Pop().has_value());
}

TEST_F(ArmRequestQueueTest, puts_an_emergency_stop_ahead_of_everything_queued)
{
    queue.Push({ 1, arm_host::ArmCommand{ arm_host::MoveTo{} } });
    queue.Push({ 2, arm_host::ArmCommand{ arm_host::MoveTo{} } });
    queue.Push({ 3, arm_host::ArmCommand{ arm_host::EmergencyStop{} } });

    EXPECT_EQ(3u, PopId());
    EXPECT_EQ(1u, PopId());
}
