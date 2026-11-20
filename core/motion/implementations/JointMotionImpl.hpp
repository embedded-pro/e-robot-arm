#ifndef CORE_MOTION_IMPLEMENTATIONS_JOINT_MOTION_IMPL_HPP
#define CORE_MOTION_IMPLEMENTATIONS_JOINT_MOTION_IMPL_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/motion/implementations/JointJogger.hpp"
#include "core/motion/implementations/JointLinePlan.hpp"
#include "core/motion/implementations/JointMovePlan.hpp"
#include "core/motion/implementations/JointPathPlan.hpp"
#include "core/motion/implementations/ToolJogger.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "core/safety/interfaces/PoseGuard.hpp"
#include "infra/timer/Timer.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <variant>

namespace arm
{
    struct ToolLineSetup
    {
        const ArmModel* model{ nullptr };
        ToolLimits limits{};
    };

    class JointMotionImpl
        : public JointMotion
    {
    public:
        static constexpr float pathSampleSpacing{ 0.05f };
        static constexpr std::size_t maxPathSamples{ 64 };

        JointMotionImpl(JointBus& bus, const JointLimits& positionLimits, const JointMotionLimits& limits, infra::Duration period, const PoseGuard* guard = nullptr,
            const JointJerkLimits& jerks = {}, const ToolLineSetup& lines = {});

        bool MoveTo(const JointPositions& target) override;
        bool StepTo(const JointPositions& target) override;
        bool MoveThrough(const JointWaypoints& waypoints) override;
        bool MoveLine(const ToolPose& target) override;
        bool Jog(const JointJog& jog) override;
        bool Jog(const ToolJog& jog) override;
        std::optional<LineRefusal> LineRefused() const override;
        void Abort() override;
        bool Moving() const override;
        bool SelectProfile(MotionProfile newProfile) override;
        MotionProfile Profile() const override;

        std::optional<JointReference> Step();

    private:
        enum class Generator : uint8_t
        {
            none,
            pointToPoint,
            step,
            path,
            line,
            jog,
            toolJog
        };

        using PendingJog = std::variant<JointJog, ToolJog>;

        bool Accepts(const JointPositions& target) const;
        bool Acceptable(const JointPositions& target) const;
        bool Admissible(const JointPositions& target) const;
        std::size_t PathPeriods() const;
        float PeriodSeconds() const;
        bool PathWithinLimits() const;
        float Elapsed() const;
        bool PathClear(const JointPositions& from, const JointPositions& to) const;
        bool WithinStep(const JointPositions& target) const;
        bool AnyReadingStale() const;
        JointPositions LatestPositions() const;
        JointReference Sampled(float elapsed) const;
        std::optional<JointReference> StepLine();
        JointReference StepJog();
        JointReference StepToolJog();
        void EndJog(const JointPositions& last);
        bool StartJog(const JointJog& jog, const JointPositions& from);
        bool StartJog(const ToolJog& jog, const JointPositions& from);
        void Arrive();
        void Finish();

        JointBus& bus;
        JointLimits positionLimits;
        JointMotionLimits limits;
        JointJerkLimits jerks;
        MotionProfile profile{ MotionProfile::trapezoid };
        infra::Duration period;
        const PoseGuard* guard;
        JointMovePlan plan;
        JointPathPlan path;
        std::optional<JointLinePlan> line;
        std::optional<LineRefusal> lineRefusal;
        JointJogger jogger;
        std::optional<ToolJogger> toolJogger;
        std::optional<PendingJog> pendingJog;
        uint32_t pendingAge{ 0 };
        JointPositions target{};
        uint32_t periodsElapsed{ 0 };
        Generator generator{ Generator::none };
    };
}

#endif
