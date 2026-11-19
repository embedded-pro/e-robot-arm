#ifndef CORE_ARM_ARM_HPP
#define CORE_ARM_ARM_HPP

#include "core/arm/ArmConfig.hpp"
#include "core/arm/ArmControllerEngagement.hpp"
#include "core/arm_controllers/implementations/ArmControllersImpl.hpp"
#include "core/arm_controllers/implementations/ComputedTorque.hpp"
#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"
#include "core/arm_controllers/implementations/GravityPid.hpp"
#include "core/collision/CollisionObserver.hpp"
#include "core/control_loop/implementations/ControlLoopImpl.hpp"
#include "core/joint_bus/implementations/JointBusImpl.hpp"
#include "core/joint_configurator/implementations/JointConfiguratorImpl.hpp"
#include "core/lifecycle/implementations/ArmLifecycleImpl.hpp"
#include "core/motion/implementations/JointMotionImpl.hpp"
#include "core/safety/implementations/SafetySupervisorImpl.hpp"
#include "core/workspace/WorkspaceCheck.hpp"
#include "e-foc-protocol/client/FocMotorCategoryClient.hpp"
#include "infra/timer/Timer.hpp"
#include <cstdint>

namespace arm
{
    JointInertias ReflectedRotors(const ArmConfig& config);
    PdGainMatrices GravityPdGains(const ArmModel& model, const ArmConfig& config);
    MotionSupervision Supervision(const ArmConfig& config);

    class Arm
    {
    public:
        Arm(can::FocMotorCategoryClient& client, const ArmConfig& config, HardwareStop* hardwareStop = nullptr);
        Arm(const Arm& other) = delete;
        Arm& operator=(const Arm& other) = delete;

        SafetySupervisor& Safety();
        const SafetySupervisor& Safety() const;
        JointMotion& Motion();
        const JointMotion& Motion() const;
        JointConfigurator& Configurator();
        const JointConfigurator& Configurator() const;
        ArmLifecycle& Lifecycle();
        const ArmLifecycle& Lifecycle() const;
        ArmControllers& Controllers();
        const ArmControllers& Controllers() const;
        const std::optional<JointTorques>& EstimatedExternalTorques() const;
        const WorkspaceCheck& Workspace() const;
        ControlLoop& Loop();

    private:
        void Step();
        std::optional<JointTorques> ExternalTorques(const std::optional<JointSetpoints>& applied, bool streaming);

        JointBusImpl jointBus;
        SafetySupervisorImpl safety;
        JointMotionImpl motion;
        JointConfiguratorImpl configurator;
        ArmLifecycleImpl lifecycle;
        ArmModel model;
        WorkspaceCheck workspace;
        GravityCompensatedPd gravityPd;
        ComputedTorque computedTorque;
        GravityPid gravityPid;
        CollisionObserver collision;
        infra::Duration period;
        bool observing{ false };
        JointTorques previousApplied{};
        JointTorques actingSum{};
        uint32_t actingPeriods{ 0 };
        std::array<uint32_t, jointCount> observedReadings{};
        infra::TimePoint observedRequest{};
        std::optional<JointTorques> externalTorques;
        ArmControllersImpl controllers;
        ArmControllerEngagement engagement;
        ControlLoopImpl loop;
    };
}

#endif
