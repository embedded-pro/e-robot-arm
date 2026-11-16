#include "cucumber_cpp/Steps.hpp"
#include "integration_tests/virtual/support/VirtualArmFixture.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>

using integration::VirtualArmFixture;
using namespace std::chrono_literals;

namespace
{
    bool InState(VirtualArmFixture& fixture, arm::ArmState state, std::chrono::milliseconds timeout)
    {
        return fixture.WaitFor([state](const arm_host::ArmSnapshot& snapshot)
            {
                return snapshot.state == state;
            },
            timeout);
    }
}

GIVEN(R"(the arm on six e-foc virtual motors)")
{
    context.Emplace<VirtualArmFixture>();
    auto& fixture = context.Get<VirtualArmFixture>();
    ASSERT_TRUE(fixture.profile.name == "virtual reference arm");
    ASSERT_EQ(std::nullopt, fixture.Start());
}

WHEN(R"(the arm is brought up and enabled)")
{
    auto& fixture = context.Get<VirtualArmFixture>();
    ASSERT_TRUE(fixture.Command(arm_host::CheckContracts{}));
    ASSERT_TRUE(InState(fixture, arm::ArmState::waitingForCalibration, 5s));
    ASSERT_TRUE(fixture.Command(arm_host::AlignJoints{}));
    ASSERT_TRUE(InState(fixture, arm::ArmState::ready, 10s));
    ASSERT_TRUE(fixture.Command(arm_host::Enable{}));
    ASSERT_TRUE(InState(fixture, arm::ArmState::enabled, 10s));
}

WHEN(R"(the arm moves the joints to {float}, {float}, {float}, {float}, {float}, {float} rad)", (float q1, float q2, float q3, float q4, float q5, float q6))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    fixture.ForgetTrace();
    const arm::JointPositions target{ foc::Radians{ q1 }, foc::Radians{ q2 }, foc::Radians{ q3 }, foc::Radians{ q4 }, foc::Radians{ q5 }, foc::Radians{ q6 } };
    ASSERT_TRUE(fixture.Command(arm_host::MoveTo{ target }));
}

THEN(R"(the move shall be done within {int} ms)", (std::int32_t milliseconds))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    EXPECT_TRUE(fixture.WaitForEvent([](const arm_host::ArmEvent& event)
        {
            return event.kind == arm_host::ArmEventKind::moveDone;
        },
        std::chrono::milliseconds{ milliseconds }));
}

THEN(R"(within {int} ms every joint shall be within {float} rad of {float}, {float}, {float}, {float}, {float}, {float})",
    (std::int32_t milliseconds, float tolerance, float q1, float q2, float q3, float q4, float q5, float q6))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    const std::array<float, arm::jointCount> target{ q1, q2, q3, q4, q5, q6 };
    EXPECT_TRUE(fixture.WaitFor([&target, tolerance](const arm_host::ArmSnapshot& snapshot)
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                if (std::abs(snapshot.readings[joint].position.Value() - target[joint]) > tolerance)
                    return false;
            return true;
        },
        std::chrono::milliseconds{ milliseconds }));
}

THEN(R"(the largest tracking error during the move shall stay below {float} rad)", (float limit))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    const auto largest = fixture.LargestTrackingError();
    std::cout << "[METRIC] virtual_arm.tracking_error_rad=" << largest << " limit=" << limit << " samples=" << fixture.trace.size() << std::endl;
    EXPECT_FALSE(fixture.trace.empty());
    EXPECT_LT(largest, limit);
}

WHEN(R"(motor {int} stops answering)", (std::int32_t motor))
{
    context.Get<VirtualArmFixture>().virtualArm->Silence(static_cast<std::size_t>(motor - 1), true);
}

THEN(R"(within {int} ms the arm shall trip with joint {int} silent)", (std::int32_t milliseconds, std::int32_t joint))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    const auto silentJoint = static_cast<std::size_t>(joint - 1);
    EXPECT_TRUE(fixture.WaitForEvent([silentJoint](const arm_host::ArmEvent& event)
        {
            return event.kind == arm_host::ArmEventKind::jointSilent && event.joint == silentJoint;
        },
        std::chrono::milliseconds{ milliseconds }));
    EXPECT_TRUE(fixture.WaitFor([](const arm_host::ArmSnapshot& snapshot)
        {
            return snapshot.tripped && snapshot.state == arm::ArmState::tripped;
        },
        1s));
}

WHEN(R"(a collision pushes joint {int} with {float} N·m)", (std::int32_t joint, float torque))
{
    context.Get<VirtualArmFixture>().virtualArm->Collide(static_cast<std::size_t>(joint - 1), foc::NewtonMeter{ torque });
}

THEN(R"(within {int} ms the arm shall trip with joint {int} faulted on a following error)", (std::int32_t milliseconds, std::int32_t joint))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    const auto faultedJoint = static_cast<std::size_t>(joint - 1);
    EXPECT_TRUE(fixture.WaitForEvent([faultedJoint](const arm_host::ArmEvent& event)
        {
            return event.kind == arm_host::ArmEventKind::jointFault && event.joint == faultedJoint && event.fault == can::FocFaultCode::followingError;
        },
        std::chrono::milliseconds{ milliseconds }));
    EXPECT_TRUE(fixture.WaitFor([](const arm_host::ArmSnapshot& snapshot)
        {
            return snapshot.tripped && snapshot.state == arm::ArmState::tripped;
        },
        1s));
}
