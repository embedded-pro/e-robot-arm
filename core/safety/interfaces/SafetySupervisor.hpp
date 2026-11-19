#ifndef CORE_SAFETY_INTERFACES_SAFETY_SUPERVISOR_HPP
#define CORE_SAFETY_INTERFACES_SAFETY_SUPERVISOR_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/safety/interfaces/HardwareStop.hpp"
#include "core/safety/interfaces/PoseGuard.hpp"
#include "e-foc-protocol/foc/Units.hpp"
#include "infra/timer/Timer.hpp"
#include <array>
#include <chrono>
#include <limits>
#include <optional>

namespace arm
{
    struct TrackingLimit
    {
        foc::Radians window{ 0.2f };
        infra::Duration time{ std::chrono::milliseconds{ 200 } };
    };

    using TrackingLimits = std::array<TrackingLimit, jointCount>;
    using CollisionLimits = std::array<foc::NewtonMeter, jointCount>;

    inline CollisionLimits DefaultCollisionLimits()
    {
        CollisionLimits limits{};
        limits.fill(foc::NewtonMeter{ 20.0f });
        return limits;
    }

    struct ReferenceLimit
    {
        foc::RadiansPerSecond speed{ std::numeric_limits<float>::infinity() };
        foc::RadiansPerSecondSquared acceleration{ std::numeric_limits<float>::infinity() };
    };

    struct MotionSupervision
    {
        std::array<ReferenceLimit, jointCount> reference{};
        std::array<foc::RadiansPerSecond, jointCount> speed{ Unbounded<foc::RadiansPerSecond>() };
        std::array<foc::NewtonMeter, jointCount> torque{ Unbounded<foc::NewtonMeter>() };

        template<class Quantity>
        static constexpr std::array<Quantity, jointCount> Unbounded()
        {
            std::array<Quantity, jointCount> limits{};
            limits.fill(Quantity{ std::numeric_limits<float>::infinity() });
            return limits;
        }
    };

    using JointFlags = std::array<bool, jointCount>;

    class SafetySupervisor
        : public JointBus
    {
    public:
        virtual bool Tripped() const = 0;
        virtual bool Holding() const = 0;
        virtual bool Rearm() = 0;
        virtual void Supervise(const std::optional<JointReference>& reference, const std::optional<JointTorques>& externalTorques) = 0;
        virtual const JointFlags& TorqueClamped() const = 0;
        virtual std::optional<WorkspaceViolation> LastWorkspaceViolation() const = 0;
        virtual bool HardwareStopOpen() const = 0;

    protected:
        ~SafetySupervisor() = default;
    };
}

#endif
