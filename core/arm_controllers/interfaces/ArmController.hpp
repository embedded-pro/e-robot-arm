#pragma once

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
        JointTorques torques{};
    };

    class ArmController
    {
    public:
        ArmController() = default;
        ArmController(const ArmController& other) = delete;
        ArmController& operator=(const ArmController& other) = delete;

        virtual bool BumplessTransfer() const = 0;
        virtual void Reset(const MeasuredJoints& measured, const JointReference& held) = 0;
        virtual JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) = 0;

    protected:
        ~ArmController() = default;
    };
}
