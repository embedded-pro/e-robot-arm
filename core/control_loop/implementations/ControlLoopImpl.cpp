#include "core/control_loop/implementations/ControlLoopImpl.hpp"
#include "infra/util/ReallyAssert.hpp"

namespace arm
{
    ControlLoopImpl::ControlLoopImpl(JointBus& bus, infra::Duration period, const infra::Function<void()>& setpointStep)
        : bus(bus)
        , halfPeriod(period / 2)
        , setpointStep(setpointStep)
    {
        really_assert(halfPeriod > infra::Duration::zero());
    }

    void ControlLoopImpl::Start()
    {
        if (Running())
            return;

        next = Phase::setpoints;
        bus.RequestTelemetry();
        timer.Start(halfPeriod, [this]()
            {
                RunPhase();
            });
    }

    void ControlLoopImpl::Stop()
    {
        timer.Cancel();
    }

    bool ControlLoopImpl::Running() const
    {
        return timer.Armed();
    }

    void ControlLoopImpl::RunPhase()
    {
        if (next == Phase::setpoints)
        {
            next = Phase::telemetry;
            setpointStep();
        }
        else
        {
            next = Phase::setpoints;
            bus.RequestTelemetry();
        }
    }
}
