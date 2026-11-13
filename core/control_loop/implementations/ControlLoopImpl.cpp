#include "core/control_loop/implementations/ControlLoopImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>

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
        due = infra::Now() + halfPeriod;
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

    uint32_t ControlLoopImpl::OverrunCount() const
    {
        return overruns;
    }

    void ControlLoopImpl::RunPhase()
    {
        CountOverruns();

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

    void ControlLoopImpl::CountOverruns()
    {
        const auto lateness = std::max(infra::Now() - due, infra::Duration::zero());
        const auto missed = static_cast<uint32_t>(lateness / halfPeriod);

        if (lateness > halfPeriod / 2)
            overruns += 1 + missed;

        due += halfPeriod * (1 + missed);
    }
}
