#ifndef CORE_MOTION_INTERFACES_JOINT_MOTION_HPP
#define CORE_MOTION_INTERFACES_JOINT_MOTION_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "e-foc-protocol/foc/Units.hpp"
#include "infra/util/BoundedVector.hpp"
#include "infra/util/Observer.hpp"
#include "numerical/math/SE3Transform.hpp"
#include <array>
#include <cstdint>
#include <optional>

namespace arm
{
    struct JointMotionLimit
    {
        foc::RadiansPerSecond maxSpeed;
        foc::RadiansPerSecondSquared maxAcceleration;
    };

    using JointMotionLimits = std::array<JointMotionLimit, jointCount>;
    using JointJerkLimits = std::array<std::optional<foc::RadiansPerSecondCubed>, jointCount>;

    struct ToolLimits
    {
        float speedMPerS{ 0.25f };
        float accelerationMPerS2{ 1.0f };
        float jerkMPerS3{ 10.0f };
        float angularSpeedRadPerS{ 1.0f };
        float angularAccelerationRadPerS2{ 4.0f };
        float angularJerkRadPerS3{ 40.0f };
        float manipulabilityMin{ 0.001f };

        bool operator==(const ToolLimits& other) const = default;
    };

    using ToolPose = math::SE3Transform<float>;

    enum class LineFault : uint8_t
    {
        unreachable,
        outsideLimits,
        nearSingularity,
        tooFast,
        workspace
    };

    struct LineRefusal
    {
        LineFault fault{ LineFault::unreachable };
        float timeS{ 0.0f };

        bool operator==(const LineRefusal& other) const = default;
    };

    struct JointJog
    {
        std::size_t joint{ 0 };
        foc::RadiansPerSecond speed{};
    };

    enum class JogFrame : uint8_t
    {
        base,
        tool
    };

    enum class ToolAxis : uint8_t
    {
        x,
        y,
        z,
        rx,
        ry,
        rz
    };

    struct ToolJog
    {
        JogFrame frame{ JogFrame::base };
        ToolAxis axis{ ToolAxis::x };
        float speed{ 0.0f };
    };

    constexpr uint32_t jogDeadManPeriods{ 20 };
    constexpr std::size_t maxWaypoints{ 8 };
    using JointWaypoints = infra::BoundedVector<JointPositions>;

    enum class MotionProfile : uint8_t
    {
        trapezoid,
        sCurve
    };

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

        static constexpr float maxStep{ maxReferenceStep };

        virtual bool MoveTo(const JointPositions& target) = 0;
        virtual bool StepTo(const JointPositions& target) = 0;
        virtual bool MoveThrough(const JointWaypoints& waypoints) = 0;
        virtual bool MoveLine(const ToolPose& target) = 0;
        virtual bool Jog(const JointJog& jog) = 0;
        virtual bool Jog(const ToolJog& jog) = 0;
        virtual std::optional<LineRefusal> LineRefused() const = 0;
        virtual void Abort() = 0;
        virtual bool Moving() const = 0;
        virtual bool SelectProfile(MotionProfile profile) = 0;
        virtual MotionProfile Profile() const = 0;

    protected:
        ~JointMotion() = default;
    };
}

#endif
