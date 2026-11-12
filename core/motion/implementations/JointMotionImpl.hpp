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
        JointMotionImpl(JointBus& bus, const JointMotionLimits& limits, infra::Duration period);

        bool MoveTo(const JointPositions& target) override;
        void Abort() override;
        bool Moving() const override;

    private:
        JointPositions LatestPositions() const;
        void Step();
        void Finish();

        JointBus& bus;
        JointMotionLimits limits;
        infra::Duration period;
        JointMovePlan plan;
        JointPositions target{};
        uint32_t periodsElapsed{ 0 };
        infra::TimerRepeating timer;
    };
}

#endif
