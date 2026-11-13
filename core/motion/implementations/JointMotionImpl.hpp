#ifndef CORE_MOTION_IMPLEMENTATIONS_JOINT_MOTION_IMPL_HPP
#define CORE_MOTION_IMPLEMENTATIONS_JOINT_MOTION_IMPL_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/motion/implementations/JointMovePlan.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "infra/timer/Timer.hpp"
#include <cstdint>

namespace arm
{
    class JointMotionImpl
        : public JointMotion
    {
    public:
        JointMotionImpl(JointBus& bus, const JointLimits& positionLimits, const JointMotionLimits& limits, infra::Duration period);

        bool MoveTo(const JointPositions& target) override;
        void Abort() override;
        bool Moving() const override;
        std::optional<JointPositions> Setpoint() const override;

        void Step();

    private:
        bool AnyReadingStale() const;
        JointPositions LatestPositions() const;
        void Finish();

        JointBus& bus;
        JointLimits positionLimits;
        JointMotionLimits limits;
        infra::Duration period;
        JointMovePlan plan;
        JointPositions target{};
        std::optional<JointPositions> setpoint;
        uint32_t periodsElapsed{ 0 };
        bool moving{ false };
    };
}

#endif
