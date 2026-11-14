#ifndef CORE_ARM_CONTROLLERS_INTERFACES_ARM_CONTROLLER_HPP
#define CORE_ARM_CONTROLLERS_INTERFACES_ARM_CONTROLLER_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"

namespace arm
{
    struct MeasuredJoints
    {
        JointPositions positions{};
        JointSpeeds speeds{};
    };

    struct JointSetpoints
    {
        JointPositions positions{};
    };

    class ArmController
    {
    public:
        ArmController() = default;
        ArmController(const ArmController& other) = delete;
        ArmController& operator=(const ArmController& other) = delete;

        virtual bool BumplessTransfer() const = 0;
        virtual void Reset(const MeasuredJoints& measured) = 0;
        virtual JointSetpoints Step(const MeasuredJoints& measured, const JointPositions& reference) = 0;

    protected:
        ~ArmController() = default;
    };
}

#endif
