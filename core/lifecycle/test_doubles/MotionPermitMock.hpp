#pragma once

#include "core/lifecycle/interfaces/ArmLifecycle.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class MotionPermitMock
        : public MotionPermit
    {
    public:
        MOCK_METHOD(bool, MotionPermitted, (), (const, override));
    };
}
