#ifndef CORE_MOTION_TEST_DOUBLES_JOINT_MOTION_MOCK_HPP
#define CORE_MOTION_TEST_DOUBLES_JOINT_MOTION_MOCK_HPP

#include "core/motion/interfaces/JointMotion.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class JointMotionMock
        : public JointMotion
    {
    public:
        MOCK_METHOD(bool, MoveTo, (const JointPositions& target), (override));
        MOCK_METHOD(void, Abort, (), (override));
        MOCK_METHOD(bool, Moving, (), (const, override));
        MOCK_METHOD(std::optional<JointPositions>, Setpoint, (), (const, override));
    };
}

#endif
