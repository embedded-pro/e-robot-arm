#include "core/control_loop/implementations/ControlLoopImpl.hpp"
#include "core/joint_bus/test_doubles/JointBusMock.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    using namespace testing;
    using namespace std::chrono_literals;

    class ControlLoopImplTest
        : public Test
        , public infra::ClockFixture
    {
    public:
        void Stall(infra::Duration duration)
        {
            systemTimerService.TimeProgressed(duration);
            ExecuteAllActions();
        }

        StrictMock<arm::JointBusMock> bus;
        StrictMock<MockFunction<void()>> setpointStep;
        arm::ControlLoopImpl loop{ bus, 10ms, [this]()
            {
                setpointStep.Call();
            } };
    };
}

TEST_F(ControlLoopImplTest, starting_requests_telemetry_at_once_and_steps_half_a_period_later)
{
    InSequence order;
    EXPECT_CALL(bus, RequestTelemetry()).WillOnce(Return(true));
    loop.Start();
    EXPECT_TRUE(loop.Running());

    ForwardTime(4ms);

    EXPECT_CALL(setpointStep, Call());
    ForwardTime(1ms);
}

TEST_F(ControlLoopImplTest, alternates_telemetry_and_setpoint_phases_every_half_period)
{
    InSequence order;
    EXPECT_CALL(bus, RequestTelemetry()).WillOnce(Return(true));
    for (int period = 0; period != 3; ++period)
    {
        EXPECT_CALL(setpointStep, Call());
        EXPECT_CALL(bus, RequestTelemetry()).WillOnce(Return(true));
    }

    loop.Start();
    ForwardTime(30ms);
}

TEST_F(ControlLoopImplTest, stopping_ends_both_phases)
{
    EXPECT_CALL(bus, RequestTelemetry()).WillOnce(Return(true));
    loop.Start();

    loop.Stop();

    EXPECT_FALSE(loop.Running());
    ForwardTime(30ms);
}

TEST_F(ControlLoopImplTest, starting_a_running_loop_does_nothing)
{
    EXPECT_CALL(bus, RequestTelemetry()).WillOnce(Return(true));
    loop.Start();

    loop.Start();
}

TEST_F(ControlLoopImplTest, a_phase_that_starts_more_than_a_quarter_period_late_is_an_overrun)
{
    EXPECT_CALL(bus, RequestTelemetry()).WillRepeatedly(Return(true));
    EXPECT_CALL(setpointStep, Call()).Times(AnyNumber());
    loop.Start();
    ForwardTime(20ms);
    EXPECT_EQ(0u, loop.OverrunCount());

    Stall(7ms);
    EXPECT_EQ(0u, loop.OverrunCount());

    ForwardTime(3ms);
    Stall(8ms);
    EXPECT_EQ(1u, loop.OverrunCount());

    ForwardTime(2ms);
    EXPECT_EQ(1u, loop.OverrunCount());
}

TEST_F(ControlLoopImplTest, a_stall_counts_every_phase_it_missed_once)
{
    EXPECT_CALL(bus, RequestTelemetry()).WillRepeatedly(Return(true));
    EXPECT_CALL(setpointStep, Call()).Times(AnyNumber());
    loop.Start();

    Stall(17ms);
    EXPECT_EQ(3u, loop.OverrunCount());

    ForwardTime(3ms);
    EXPECT_EQ(3u, loop.OverrunCount());

    loop.Stop();
    loop.Start();
    EXPECT_EQ(3u, loop.OverrunCount());
}
