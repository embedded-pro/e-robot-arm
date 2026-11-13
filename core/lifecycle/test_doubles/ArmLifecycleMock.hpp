#ifndef CORE_LIFECYCLE_TEST_DOUBLES_ARM_LIFECYCLE_MOCK_HPP
#define CORE_LIFECYCLE_TEST_DOUBLES_ARM_LIFECYCLE_MOCK_HPP

#include "core/lifecycle/interfaces/ArmLifecycle.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class ArmLifecycleMock
        : public ArmLifecycle
    {
    public:
        MOCK_METHOD(bool, BringUp, (), (override));
        MOCK_METHOD(bool, Enable, (), (override));
        MOCK_METHOD(bool, Disable, (), (override));
        MOCK_METHOD(bool, Rearm, (), (override));
        MOCK_METHOD(bool, MoveTo, (const JointPositions& target), (override));
        MOCK_METHOD(ArmState, State, (), (const, override));
    };

    class ArmLifecycleObserverMock
        : public ArmLifecycleObserver
    {
    public:
        using ArmLifecycleObserver::ArmLifecycleObserver;

        MOCK_METHOD(void, OnStateChanged, (ArmState state), (override));
    };
}

#endif
