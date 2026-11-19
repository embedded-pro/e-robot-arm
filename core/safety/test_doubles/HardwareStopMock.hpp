#ifndef CORE_SAFETY_TEST_DOUBLES_HARDWARE_STOP_MOCK_HPP
#define CORE_SAFETY_TEST_DOUBLES_HARDWARE_STOP_MOCK_HPP

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

#endif
