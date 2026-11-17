#ifndef CORE_SAFETY_INTERFACES_SAFETY_SUPERVISOR_HPP
#define CORE_SAFETY_INTERFACES_SAFETY_SUPERVISOR_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "e-foc-protocol/foc/Units.hpp"
#include "infra/timer/Timer.hpp"
#include <array>
#include <chrono>
#include <optional>

namespace arm
{
    struct TrackingLimit
    {
        foc::Radians window{ 0.2f };
        infra::Duration time{ std::chrono::milliseconds{ 200 } };
    };

    using TrackingLimits = std::array<TrackingLimit, jointCount>;

    class SafetySupervisor
        : public JointBus
    {
    public:
        virtual bool Tripped() const = 0;
        virtual bool Rearm() = 0;
        virtual void Supervise(const std::optional<JointPositions>& reference) = 0;

    protected:
        ~SafetySupervisor() = default;
    };
}

#endif
