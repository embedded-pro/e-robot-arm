#ifndef CORE_ARM_ARM_HPP
#define CORE_ARM_ARM_HPP

#include "core/control_loop/implementations/ControlLoopImpl.hpp"
#include "core/joint_bus/implementations/JointBusImpl.hpp"
#include "core/lifecycle/implementations/ArmLifecycleImpl.hpp"
#include "core/motion/implementations/JointMotionImpl.hpp"
#include "core/safety/implementations/SafetySupervisorImpl.hpp"
#include "e-foc-protocol/client/FocMotorCategoryClient.hpp"
#include "infra/timer/Timer.hpp"
#include <cstdint>

namespace arm
{
    struct ArmConfig
    {
        JointNodeIds nodes;
        uint8_t missedRequestsBeforeSilent;
        JointLimits positionLimits;
        JointMotionLimits motionLimits;
        infra::Duration controlPeriod;
        uint8_t commandTimeoutPeriods{ 3 };
        JointTransmissions transmissions{};
        can::FocMotorMode enableMode{ can::FocMotorMode::position };
    };

    class Arm
    {
    public:
        Arm(can::FocMotorCategoryClient& client, const ArmConfig& config);
        Arm(const Arm& other) = delete;
        Arm& operator=(const Arm& other) = delete;

        SafetySupervisor& Safety();
        const SafetySupervisor& Safety() const;
        JointMotion& Motion();
        const JointMotion& Motion() const;
        ArmLifecycle& Lifecycle();
        const ArmLifecycle& Lifecycle() const;
        ControlLoop& Loop();

    private:
        JointBusImpl jointBus;
        SafetySupervisorImpl safety;
        JointMotionImpl motion;
        ArmLifecycleImpl lifecycle;
        ControlLoopImpl loop;
    };
}

#endif
