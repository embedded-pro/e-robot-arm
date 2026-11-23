#include "cucumber_cpp/Steps.hpp"
#include "integration_tests/support/ArmFixture.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <map>
#include <string>

using integration::ArmFixture;
using namespace std::chrono_literals;

namespace
{
    constexpr std::int32_t samplePeriodMs{ 10 };
    constexpr float torqueTolerance{ 1.0e-3f };

    arm::SupervisionTrip TripNamed(const std::string& name)
    {
        static const std::map<std::string, arm::SupervisionTrip> trips{ { "speed", arm::SupervisionTrip::speedLimit },
            { "reference speed", arm::SupervisionTrip::referenceSpeed }, { "reference acceleration", arm::SupervisionTrip::referenceAcceleration },
            { "reference step", arm::SupervisionTrip::referenceStep }, { "tracking error", arm::SupervisionTrip::trackingError },
            { "position limit", arm::SupervisionTrip::positionLimit } };
        return trips.at(name);
    }

    arm::JointPositions Measured(ArmFixture& fixture)
    {
        arm::JointPositions positions{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            positions[joint] = fixture.arm.Safety().Reading(joint).position;
        return positions;
    }
}

GIVEN(R"(an arm with six simulated joints whose joint {int} may turn at most {float} radians per second)", (std::int32_t joint, float speed))
{
    auto config = ArmFixture::DefaultConfig();
    config.speedTrips[static_cast<std::size_t>(joint - 1)] = foc::RadiansPerSecond{ speed };
    context.Emplace<ArmFixture>(config);
}

GIVEN(R"(an arm with six simulated joints whose joint {int} may drive at most {float} N·m)", (std::int32_t joint, float torque))
{
    auto config = ArmFixture::DefaultConfig();
    config.torqueLimits[static_cast<std::size_t>(joint - 1)] = foc::NewtonMeter{ torque };
    context.Emplace<ArmFixture>(config);
}

GIVEN(R"(an arm with six simulated joints whose joint {int} may move from {float} to {float} rad)", (std::int32_t joint, float lower, float upper))
{
    auto config = ArmFixture::DefaultConfig();
    config.positionLimits[static_cast<std::size_t>(joint - 1)] = { foc::Radians{ lower }, foc::Radians{ upper } };
    context.Emplace<ArmFixture>(config);
}

WHEN(R"(joint {int} is pushed with {float} N·m)", (std::int32_t joint, float torque))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.simulatedArm.Push(static_cast<std::size_t>(joint - 1), foc::NewtonMeter{ torque });
}

WHEN(R"(the push on joint {int} ends)", (std::int32_t joint))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.simulatedArm.Push(static_cast<std::size_t>(joint - 1), foc::NewtonMeter{ 0.0f });
}

THEN(R"(the arm shall report joint {int} measured beyond {float} rad)", (std::int32_t joint, float limit))
{
    auto& fixture = context.Get<ArmFixture>();
    ASSERT_TRUE(fixture.events.protectiveStop.has_value());
    EXPECT_EQ(static_cast<std::size_t>(joint - 1), fixture.events.protectiveStop->first);
    std::cout << "[METRIC] supervision.position_limit_stop_rad=" << fixture.events.protectiveStopValue << " joint=" << joint << " limit=" << limit << "\n";
    EXPECT_GT(std::abs(fixture.events.protectiveStopValue), std::abs(limit));
}

WHEN(R"(the arm moves joint {int} {float} rad further)", (std::int32_t joint, float distance))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.target = Measured(fixture);
    auto& moved = fixture.target[static_cast<std::size_t>(joint - 1)];
    moved = foc::Radians{ moved.Value() + distance };
    fixture.movesDoneBefore = fixture.events.movesDone;
    EXPECT_TRUE(fixture.arm.Lifecycle().MoveTo(fixture.target));
}

WHEN(R"(the arm is re-armed)")
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_TRUE(fixture.arm.Lifecycle().Rearm());
    fixture.ForwardTime(30ms);
}

WHEN(R"(the arm is handed a reference that drives joint {int} at {float} radians per second)", (std::int32_t joint, float speed))
{
    auto& fixture = context.Get<ArmFixture>();
    auto reference = arm::AtRest(Measured(fixture));
    reference.speeds[static_cast<std::size_t>(joint - 1)] = foc::RadiansPerSecond{ speed };
    fixture.arm.Safety().Supervise(reference, std::nullopt);
    fixture.ForwardTime(20ms);
}

THEN(R"(within {int} ms the arm shall stop protectively, naming joint {int} on its {string})", (std::int32_t milliseconds, std::int32_t joint, const std::string& rule))
{
    auto& fixture = context.Get<ArmFixture>();
    for (std::int32_t elapsed = 0; elapsed < milliseconds && !fixture.events.protectiveStop.has_value(); elapsed += samplePeriodMs)
        fixture.ForwardTime(std::chrono::milliseconds{ samplePeriodMs });

    ASSERT_TRUE(fixture.events.protectiveStop.has_value());
    EXPECT_EQ(static_cast<std::size_t>(joint - 1), fixture.events.protectiveStop->first);
    EXPECT_EQ(TripNamed(rule), fixture.events.protectiveStop->second);
    EXPECT_EQ(arm::ArmState::holding, fixture.arm.Lifecycle().State());
    EXPECT_FALSE(fixture.arm.Safety().Tripped());
}

THEN(R"(for {int} ms every joint shall stay running within {float} rad of where the arm stopped)", (std::int32_t milliseconds, float tolerance))
{
    auto& fixture = context.Get<ArmFixture>();
    const auto stopped = Measured(fixture);
    for (std::int32_t elapsed = 0; elapsed < milliseconds; elapsed += samplePeriodMs)
    {
        fixture.ForwardTime(std::chrono::milliseconds{ samplePeriodMs });
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto& reading = fixture.arm.Safety().Reading(joint);
            EXPECT_EQ(can::FocMotorState::running, reading.state) << "joint " << joint + 1;
            EXPECT_NEAR(stopped[joint].Value(), reading.position.Value(), tolerance) << "joint " << joint + 1 << " after " << elapsed << " ms";
        }
    }
    EXPECT_EQ(arm::ArmState::holding, fixture.arm.Lifecycle().State());
}

THEN(R"(for {int} ms joint {int} shall drive at most {float} N·m, and the arm shall report its torque clamped)", (std::int32_t milliseconds, std::int32_t joint, float limit))
{
    auto& fixture = context.Get<ArmFixture>();
    const auto index = static_cast<std::size_t>(joint - 1);
    bool clamped{ false };
    float largest{ 0.0f };
    for (std::int32_t elapsed = 0; elapsed < milliseconds; elapsed += samplePeriodMs)
    {
        fixture.ForwardTime(std::chrono::milliseconds{ samplePeriodMs });
        if (const auto torque = fixture.Joint(joint).DrivingTorque(); torque.has_value())
            largest = std::max(largest, std::abs(torque->Value()));
        clamped = clamped || fixture.arm.Safety().TorqueClamped()[index];
    }
    std::cout << "[METRIC] supervision.largest_torque_nm=" << largest << " joint=" << joint << " limit=" << limit << "\n";
    EXPECT_LE(largest, limit + torqueTolerance);
    EXPECT_TRUE(clamped);
}
