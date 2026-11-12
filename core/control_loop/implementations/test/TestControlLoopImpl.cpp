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
