#ifndef CORE_MOTION_INTERFACES_JOINT_MOTION_HPP
#define CORE_MOTION_INTERFACES_JOINT_MOTION_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "e-foc-protocol/foc/Units.hpp"
#include "infra/util/Observer.hpp"
#include <array>
#include <optional>

namespace arm
{
    struct JointMotionLimit
    {
        foc::RadiansPerSecond maxSpeed;
        foc::RadiansPerSecondSquared maxAcceleration;
    };

    using JointMotionLimits = std::array<JointMotionLimit, jointCount>;

    class JointMotion;

    class JointMotionObserver
        : public infra::Observer<JointMotionObserver, JointMotion>
    {
    public:
        using infra::Observer<JointMotionObserver, JointMotion>::Observer;

        virtual void OnMoveDone() = 0;
        virtual void OnMoveAborted() = 0;

    protected:
        ~JointMotionObserver() = default;
    };

    class JointMotion
        : public infra::Subject<JointMotionObserver>
    {
    public:
        JointMotion() = default;
        JointMotion(const JointMotion& other) = delete;
        JointMotion& operator=(const JointMotion& other) = delete;

        virtual bool MoveTo(const JointPositions& target) = 0;
        virtual void Abort() = 0;
        virtual bool Moving() const = 0;
        virtual std::optional<JointPositions> Setpoint() const = 0;

    protected:
        ~JointMotion() = default;
    };
}

#endif
