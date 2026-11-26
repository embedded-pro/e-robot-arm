#pragma once

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "infra/util/Observer.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>

namespace arm
{
    enum class ArmState : uint8_t
    {
        unpowered,
        checking,
        incompatible,
        waitingForCalibration,
        configuring,
        misconfigured,
        ready,
        enabling,
        enabled,
        moving,
        disabling,
        holding,
        tripped
    };

    enum class MisconfigurationReason : uint8_t
    {
        failed,
        differs
    };

    enum class ConfigurationPart : uint8_t
    {
        motor,
        controller,
        watchdog
    };

    struct Misconfiguration
    {
        std::size_t joint;
        MisconfigurationReason reason;
        ConfigurationPart part{ ConfigurationPart::controller };

        bool operator==(const Misconfiguration& other) const = default;
    };

    class MotionPermit
    {
    public:
        virtual bool MotionPermitted() const = 0;

    protected:
        ~MotionPermit() = default;
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
        virtual bool StepTo(const JointPositions& target) = 0;
        virtual bool MoveThrough(const JointWaypoints& waypoints) = 0;
        virtual bool MoveLine(const ToolPose& target) = 0;
        virtual bool Jog(const JointJog& jog) = 0;
        virtual bool Jog(const ToolJog& jog) = 0;
        virtual ArmState State() const = 0;
        virtual std::optional<Misconfiguration> MisconfiguredJoint() const = 0;
        virtual std::optional<std::size_t> UnstartedJoint() const = 0;

    protected:
        ~ArmLifecycle() = default;
    };
}
