#ifndef CORE_MOTION_IMPLEMENTATIONS_JOINT_MOTION_IMPL_HPP
#define CORE_MOTION_IMPLEMENTATIONS_JOINT_MOTION_IMPL_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/motion/implementations/JointMovePlan.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "core/safety/interfaces/PoseGuard.hpp"
#include "infra/timer/Timer.hpp"
#include <cstdint>
#include <optional>

namespace arm
{
    class JointMotionImpl
        : public JointMotion
    {
    public:
        static constexpr float pathSampleSpacing{ 0.05f };
        static constexpr std::size_t maxPathSamples{ 64 };

        JointMotionImpl(JointBus& bus, const JointLimits& positionLimits, const JointMotionLimits& limits, infra::Duration period, const PoseGuard* guard = nullptr);

        bool MoveTo(const JointPositions& target) override;
        bool StepTo(const JointPositions& target) override;
        void Abort() override;
        bool Moving() const override;

        std::optional<JointReference> Step();

    private:
        bool Accepts(const JointPositions& target) const;
        bool PathClear(const JointPositions& from, const JointPositions& to) const;
        bool WithinStep(const JointPositions& target) const;
        bool AnyReadingStale() const;
        JointPositions LatestPositions() const;
        void Finish();

        JointBus& bus;
        JointLimits positionLimits;
        JointMotionLimits limits;
        infra::Duration period;
        const PoseGuard* guard;
        JointMovePlan plan;
        JointPositions target{};
        uint32_t periodsElapsed{ 0 };
        bool moving{ false };
        bool stepping{ false };
    };
}

#endif
