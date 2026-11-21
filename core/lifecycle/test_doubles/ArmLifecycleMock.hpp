#pragma once

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
        MOCK_METHOD(bool, StepTo, (const JointPositions& target), (override));
        MOCK_METHOD(bool, MoveThrough, (const JointWaypoints& waypoints), (override));
        MOCK_METHOD(bool, MoveLine, (const ToolPose& target), (override));
        MOCK_METHOD(bool, Jog, (const JointJog& jog), (override));
        MOCK_METHOD(bool, Jog, (const ToolJog& jog), (override));
        MOCK_METHOD(ArmState, State, (), (const, override));
        MOCK_METHOD(std::optional<Misconfiguration>, MisconfiguredJoint, (), (const, override));
        MOCK_METHOD(std::optional<std::size_t>, UnstartedJoint, (), (const, override));
    };

    class ArmLifecycleObserverMock
        : public ArmLifecycleObserver
    {
    public:
        using ArmLifecycleObserver::ArmLifecycleObserver;

        MOCK_METHOD(void, OnStateChanged, (ArmState state), (override));
    };
}
