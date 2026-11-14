#ifndef CORE_ARM_CONTROLLERS_TEST_DOUBLES_ARM_CONTROLLER_MOCK_HPP
#define CORE_ARM_CONTROLLERS_TEST_DOUBLES_ARM_CONTROLLER_MOCK_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class ArmControllerMock
        : public ArmController
    {
    public:
        MOCK_METHOD(bool, BumplessTransfer, (), (const, override));
        MOCK_METHOD(void, Reset, (const MeasuredJoints& measured), (override));
        MOCK_METHOD(JointSetpoints, Step, (const MeasuredJoints& measured, const JointPositions& reference), (override));
    };
}

#endif
