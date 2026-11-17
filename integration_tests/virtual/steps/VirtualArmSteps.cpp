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
        const auto reached = fixture.WaitFor([state](const arm_host::ArmSnapshot& snapshot)
            {
                return snapshot.state == state;
            },
            timeout);
        if (!reached)
            std::cout << "waited " << timeout.count() << " ms for " << arm_host::StateName(state) << "; " << fixture.History() << std::flush;
        return reached;
    }
}

GIVEN(R"(the arm on six e-foc virtual motors)")
{
    context.Emplace<VirtualArmFixture>();
    auto& fixture = context.Get<VirtualArmFixture>();
    ASSERT_TRUE(fixture.profile.name == "virtual reference arm");
    ASSERT_EQ(std::nullopt, fixture.Start());
}

GIVEN(R"(the arm on six e-foc virtual motors with every joint's collision limit at {float} N·m)", (float limit))
{
    context.Emplace<VirtualArmFixture>();
    auto& fixture = context.Get<VirtualArmFixture>();
    fixture.profile.config.collision.fill(foc::NewtonMeter{ limit });
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

WHEN(R"(the arm controls its joints with {string})", (const std::string& name))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    std::optional<arm::ArmControllerId> id;
    ASSERT_TRUE(fixture.WaitFor([&name, &id](const arm_host::ArmSnapshot& snapshot)
        {
            const auto found = std::ranges::find(snapshot.armControllers, name, &arm_host::ArmControllerStatus::name);
            if (found != snapshot.armControllers.end())
                id = static_cast<arm::ArmControllerId>(std::distance(snapshot.armControllers.begin(), found));
            return id.has_value();
        },
        1s))
        << name;
    ASSERT_TRUE(fixture.Command(arm_host::SelectArmController{ *id }));
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
    const bool done = fixture.WaitForEvent([](const arm_host::ArmEvent& event)
        {
            return event.kind == arm_host::ArmEventKind::moveDone;
        },
        std::chrono::milliseconds{ milliseconds });
    EXPECT_TRUE(done) << fixture.History();
}

THEN(R"(within {int} ms every joint shall be within {float} rad of {float}, {float}, {float}, {float}, {float}, {float})",
    (std::int32_t milliseconds, float tolerance, float q1, float q2, float q3, float q4, float q5, float q6))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    const std::array<float, arm::jointCount> target{ q1, q2, q3, q4, q5, q6 };
    std::array<float, arm::jointCount> errors{};
    const bool settled = fixture.WaitFor([&target, &errors, tolerance](const arm_host::ArmSnapshot& snapshot)
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                errors[joint] = std::abs(snapshot.readings[joint].position.Value() - target[joint]);
            return std::ranges::all_of(errors, [tolerance](float error)
                {
                    return error <= tolerance;
                });
        },
        std::chrono::milliseconds{ milliseconds });
    const auto largest = std::ranges::max_element(errors);
    std::cout << "[METRIC] virtual_arm.settled_error_rad=" << *largest << " joint=" << std::distance(errors.begin(), largest) + 1 << " limit=" << tolerance
              << std::endl;
    EXPECT_TRUE(settled);
}

THEN(R"(the largest tracking error during the move shall stay below {float} rad)", (float limit))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    const auto largest = fixture.LargestTrackingError();
    std::cout << "[METRIC] virtual_arm.tracking_error_rad=" << largest << " limit=" << limit << " samples=" << fixture.trace.size() << std::endl;
    EXPECT_FALSE(fixture.trace.empty());
    EXPECT_LT(largest, limit);
}

THEN(R"(the largest external torque estimated during the move shall stay below {float} N·m)", (float limit))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    const auto [largest, joint] = fixture.LargestExternalTorque();
    std::cout << "[METRIC] virtual_arm.external_torque_nm=" << largest << " joint=" << joint + 1 << " limit=" << limit << std::endl;
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

THEN(R"(within {int} ms the arm shall trip with joint {int} on its tracking error)", (std::int32_t milliseconds, std::int32_t joint))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    const auto heldJoint = static_cast<std::size_t>(joint - 1);
    const bool tripped = fixture.WaitForEvent([heldJoint](const arm_host::ArmEvent& event)
        {
            return event.kind == arm_host::ArmEventKind::supervisionTrip && event.joint == heldJoint && event.reason.starts_with("tracking error");
        },
        std::chrono::milliseconds{ milliseconds });
    EXPECT_TRUE(tripped) << fixture.History();
    EXPECT_TRUE(fixture.WaitFor([](const arm_host::ArmSnapshot& snapshot)
        {
            return snapshot.tripped && snapshot.state == arm::ArmState::tripped;
        },
        1s));
}

THEN(R"(within {int} ms the arm shall trip with joint {int} on a collision)", (std::int32_t milliseconds, std::int32_t joint))
{
    auto& fixture = context.Get<VirtualArmFixture>();
    const auto heldJoint = static_cast<std::size_t>(joint - 1);
    const bool tripped = fixture.WaitForEvent([heldJoint](const arm_host::ArmEvent& event)
        {
            return event.kind == arm_host::ArmEventKind::supervisionTrip && event.joint == heldJoint && event.reason.starts_with("collision");
        },
        std::chrono::milliseconds{ milliseconds });
    EXPECT_TRUE(tripped) << fixture.History();
    EXPECT_TRUE(fixture.WaitFor([](const arm_host::ArmSnapshot& snapshot)
        {
            return snapshot.tripped && snapshot.state == arm::ArmState::tripped;
        },
        1s));
}
