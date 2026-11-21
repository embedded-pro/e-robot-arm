#pragma once

#include "core/safety/interfaces/HardwareStop.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class HardwareStopMock
        : public HardwareStop
    {
    public:
        MOCK_METHOD(bool, Open, (), (const, override));
    };
}
