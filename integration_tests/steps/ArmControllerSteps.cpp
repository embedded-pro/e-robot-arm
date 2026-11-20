#include "cucumber_cpp/Steps.hpp"
#include "integration_tests/support/ArmFixture.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>

using integration::ArmFixture;
using namespace std::chrono_literals;

namespace
{
    std::optional<arm::ArmControllerId> Named(ArmFixture& fixture, const std::string& name)
    {
        const auto id = fixture.arm.Controllers().Find(infra::BoundedConstString{ name.data(), name.size() });
        EXPECT_TRUE(id.has_value()) << name;
        return id;
    }
}

GIVEN(R"(an arm with six simulated joints {word} experimental controllers)", (const std::string& allowing))
{
    auto config = ArmFixture::DefaultConfig();
    config.experimental = allowing == "with";
    context.Emplace<ArmFixture>(config);
}

WHEN(R"({int} ms pass)", (std::int32_t milliseconds))
{
    context.Get<ArmFixture>().ForwardTime(std::chrono::milliseconds{ milliseconds });
}

WHEN(R"(the arm selects the {string} arm controller)", (const std::string& name))
{
    auto& fixture = context.Get<ArmFixture>();
    const auto id = Named(fixture, name);
    fixture.lastAccepted = id.has_value() && fixture.arm.Controllers().Select(*id);
}

WHEN(R"(the arm disables the {string} arm controller)", (const std::string& name))
{
    auto& fixture = context.Get<ArmFixture>();
    const auto id = Named(fixture, name);
    EXPECT_TRUE(id.has_value() && fixture.arm.Controllers().Disable(*id));
}

THEN(R"(the active arm controller shall be {string})", (const std::string& name))
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_EQ(Named(fixture, name), fixture.arm.Controllers().Active());
}

THEN(R"(no arm controller shall be active)")
{
    EXPECT_FALSE(context.Get<ArmFixture>().arm.Controllers().Active().has_value());
}

THEN(R"(disabling the {string} arm controller shall be refused)", (const std::string& name))
{
    auto& fixture = context.Get<ArmFixture>();
    const auto id = Named(fixture, name);
    ASSERT_TRUE(id.has_value());
    EXPECT_FALSE(fixture.arm.Controllers().Disable(*id));
    EXPECT_EQ(id, fixture.arm.Controllers().Active());
}

WHEN(R"(the arm is disabled)")
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_TRUE(fixture.arm.Lifecycle().Disable());
    fixture.ForwardTime(30ms);
    EXPECT_EQ(arm::ArmState::ready, fixture.arm.Lifecycle().State());
}

THEN(R"(a second after the move every joint shall be within {float} rad of its target)", (float tolerance))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.ForwardTime(1s);
    float largest{ 0.0f };
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        largest = std::max(largest, std::abs(fixture.target[joint].Value() - fixture.arm.Safety().Reading(joint).position.Value()));
        EXPECT_NEAR(fixture.target[joint].Value(), fixture.arm.Safety().Reading(joint).position.Value(), tolerance) << "joint " << joint + 1;
    }
    std::cout << "[METRIC] arm_controllers.settled_error_rad=" << largest << " limit=" << tolerance << std::endl;
}

THEN(R"(for {int} ms every joint shall stay within {float} rad of where it was)", (std::int32_t milliseconds, float tolerance))
{
    auto& fixture = context.Get<ArmFixture>();
    arm::JointPositions held{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        held[joint] = fixture.arm.Safety().Reading(joint).position;

    float largest{ 0.0f };
    for (std::int32_t elapsed = 0; elapsed < milliseconds; elapsed += 10)
    {
        fixture.ForwardTime(10ms);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            largest = std::max(largest, std::abs(fixture.arm.Safety().Reading(joint).position.Value() - held[joint].Value()));
    }
    std::cout << "[METRIC] arm_controllers.switch_drift_rad=" << largest << " limit=" << tolerance << std::endl;
    EXPECT_LE(largest, tolerance);
}
