#ifndef CORE_CONTROL_LOOP_IMPLEMENTATIONS_CONTROL_LOOP_IMPL_HPP
#define CORE_CONTROL_LOOP_IMPLEMENTATIONS_CONTROL_LOOP_IMPL_HPP

#include "core/control_loop/interfaces/ControlLoop.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "infra/timer/Timer.hpp"
#include "infra/util/Function.hpp"
#include <cstdint>

namespace arm
{
    class ControlLoopImpl
        : public ControlLoop
    {
    public:
        ControlLoopImpl(JointBus& bus, infra::Duration period, const infra::Function<void()>& setpointStep);

        void Start() override;
        void Stop() override;
        bool Running() const override;
        uint32_t OverrunCount() const override;

    private:
        enum class Phase : uint8_t
        {
            telemetry,
            setpoints
        };

        void RunPhase();
        void CountOverruns();

        JointBus& bus;
        infra::Duration halfPeriod;
        infra::Function<void()> setpointStep;
        Phase next{ Phase::setpoints };
        infra::TimePoint due{};
        uint32_t overruns{ 0 };
        infra::TimerRepeating timer;
    };
}

#endif
