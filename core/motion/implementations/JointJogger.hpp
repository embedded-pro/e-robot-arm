#pragma once

#include "core/motion/interfaces/JointMotion.hpp"
#include <cstdint>
#include <optional>

namespace arm
{
    class JointJogger
    {
    public:
        JointJogger(const JointLimits& positionLimits, const JointMotionLimits& limits, float period);

        void Start(const JointPositions& from, const JointJog& jog);
        void Command(const JointJog& jog);
        void Stop();
        JointReference Advance();
        bool Done() const;

    private:
        float TargetSpeed() const;
        float StoppingSpeed(float direction) const;

        JointLimits positionLimits;
        JointMotionLimits limits;
        float period;
        JointPositions positions{};
        std::size_t joint{ 0 };
        float speed{ 0.0f };
        float commanded{ 0.0f };
        uint32_t silentPeriods{ 0 };
        std::optional<JointJog> pending;
        bool done{ false };
    };
}
