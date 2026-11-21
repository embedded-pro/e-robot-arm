#pragma once

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class ArmControllerMock
        : public ArmController
    {
    public:
        MOCK_METHOD(bool, BumplessTransfer, (), (const, override));
        MOCK_METHOD(void, Reset, (const MeasuredJoints& measured, const JointReference& held), (override));
        MOCK_METHOD(JointSetpoints, Step, (const MeasuredJoints& measured, const JointReference& reference), (override));
    };
}
