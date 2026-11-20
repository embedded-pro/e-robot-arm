#ifndef CORE_MOTION_IMPLEMENTATIONS_TOOL_JOGGER_HPP
#define CORE_MOTION_IMPLEMENTATIONS_TOOL_JOGGER_HPP

#include "core/arm_model/ArmModel.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "core/safety/interfaces/PoseGuard.hpp"
#include <cstdint>
#include <optional>

namespace arm
{
    class ToolJogger
    {
    public:
        ToolJogger(const ArmModel& model, const ToolLimits& tool, const JointMotionLimits& joints, const PoseGuard* guard, float period);

        void Start(const JointPositions& from, const ToolJog& jog);
        void Command(const ToolJog& jog);
        void Stop();
        JointReference Advance();
        bool Done() const;

    private:
        struct State
        {
            ToolPose pose{};
            JointPositions joints{};
            JointSpeeds speeds{};
            JointAccelerations accelerations{};
            float speed{ 0.0f };
        };

        struct SpeedRange
        {
            float low;
            float high;

            void Narrow(float gain, float lower, float upper);
        };

        bool Linear() const;
        float SpeedLimit() const;
        float AccelerationLimit() const;
        float TargetSpeed() const;
        ToolPose Moved(const ToolPose& from, float distance) const;
        std::optional<State> StepFrom(const State& from, float target) const;
        std::optional<float> FeasibleSpeed(const State& from, float target) const;
        bool Acceptable(const JointPositions& joints) const;
        bool Stoppable(State from) const;
        JointReference Hold();

        const ArmModel& model;
        ToolLimits tool;
        JointMotionLimits joints;
        const PoseGuard* guard;
        float period;
        ToolJog jog{};
        State state{};
        uint32_t silentPeriods{ 0 };
        std::optional<ToolJog> pending;
        bool done{ false };
    };
}

#endif
