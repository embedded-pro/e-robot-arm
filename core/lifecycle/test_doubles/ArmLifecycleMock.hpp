#ifndef CORE_LIFECYCLE_TEST_DOUBLES_ARM_LIFECYCLE_MOCK_HPP
#define CORE_LIFECYCLE_TEST_DOUBLES_ARM_LIFECYCLE_MOCK_HPP

#include "core/lifecycle/interfaces/ArmLifecycle.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class ArmLifecycleObserverMock
        : public ArmLifecycleObserver
    {
    public:
        using ArmLifecycleObserver::ArmLifecycleObserver;

        MOCK_METHOD(void, OnStateChanged, (ArmState state), (override));
    };
}

#endif
