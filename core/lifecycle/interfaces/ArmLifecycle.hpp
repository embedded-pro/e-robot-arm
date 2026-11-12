#ifndef CORE_LIFECYCLE_INTERFACES_ARM_LIFECYCLE_HPP
#define CORE_LIFECYCLE_INTERFACES_ARM_LIFECYCLE_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "infra/util/Observer.hpp"
#include <cstdint>

namespace arm
{
    enum class ArmState : uint8_t
    {
        unpowered,
        checking,
        incompatible,
        waitingForCalibration,
        ready,
        enabling,
        enabled,
        moving,
        disabling,
        tripped
    };

    class ArmLifecycle;

    class ArmLifecycleObserver
        : public infra::Observer<ArmLifecycleObserver, ArmLifecycle>
    {
    public:
        using infra::Observer<ArmLifecycleObserver, ArmLifecycle>::Observer;

        virtual void OnStateChanged(ArmState state) = 0;

    protected:
        ~ArmLifecycleObserver() = default;
    };

    class ArmLifecycle
        : public infra::Subject<ArmLifecycleObserver>
    {
    public:
        ArmLifecycle() = default;
        ArmLifecycle(const ArmLifecycle& other) = delete;
        ArmLifecycle& operator=(const ArmLifecycle& other) = delete;

        virtual bool BringUp() = 0;
        virtual bool Enable() = 0;
        virtual bool Disable() = 0;
        virtual bool Rearm() = 0;
        virtual bool MoveTo(const JointPositions& target) = 0;
        virtual ArmState State() const = 0;

    protected:
        ~ArmLifecycle() = default;
    };
}

#endif
