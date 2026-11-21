#pragma once

#include "core/platform_abstraction/Platform.hpp"
#include "gmock/gmock.h"

namespace platform
{
    class PlatformMock
        : public Platform
    {
    public:
        // Test-only type, so the vtable entry costs nothing on target. It cannot be `final`
        // because testing::StrictMock<> derives from it, and it cannot take the protected
        // non-virtual destructor the interfaces use because tests instantiate it directly.
        virtual ~PlatformMock() = default;

        MOCK_METHOD(hal::GpioPin&, StatusLed, (), (override));
        MOCK_METHOD(hal::SerialCommunication&, Communication, (), (override));
        MOCK_METHOD(services::Tracer&, Tracer, (), (override));
        MOCK_METHOD(void, Run, (), (override));
    };
}
