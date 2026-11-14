#include "cucumber_cpp/Steps.hpp"
#include "integration_tests/support/ArmFixture.hpp"
#include <algorithm>
#include <chrono>
#include <gtest/gtest.h>

using integration::ArmFixture;
using namespace std::chrono_literals;

namespace
{
    constexpr float positionTolerance{ 1.0e-3f };
    constexpr auto controlPeriod{ 10ms };

    std::optional<arm::ArmControllerId> Named(ArmFixture& fixture, const std::string& name)
    {
        const auto id = fixture.arm.Controllers().Find(infra::BoundedConstString{ name.data(), name.size() });
        EXPECT_TRUE(id.has_value()) << name;
        return id;
    }

    arm::JointPositions JointPositions(ArmFixture& fixture)
    {
        arm::JointPositions positions{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            positions[joint] = fixture.Joint(joint + 1).Position();
        return positions;
    }
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
    fixture.movesAbortedBefore = fixture.events.movesAborted;
    fixture.lastAccepted = id.has_value() && fixture.arm.Controllers().Disable(*id);
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

THEN(R"(selecting the {string} arm controller shall be refused)", (const std::string& name))
{
    auto& fixture = context.Get<ArmFixture>();
    const auto id = Named(fixture, name);
    ASSERT_TRUE(id.has_value());
    EXPECT_FALSE(fixture.arm.Controllers().Select(*id));
}

THEN(R"(for {int} ms no joint shall move more than {float} rad in one control period)", (std::int32_t milliseconds, float step))
{
    auto& fixture = context.Get<ArmFixture>();
    auto previous = JointPositions(fixture);
    for (std::int32_t elapsed = 0; elapsed < milliseconds; elapsed += 10)
    {
        fixture.ForwardTime(controlPeriod);
        const auto now = JointPositions(fixture);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            EXPECT_LE(std::abs(now[joint].Value() - previous[joint].Value()), step) << "joint " << joint + 1 << " at " << elapsed << " ms";
        previous = now;
    }
}

THEN(R"(within {int} ms every joint shall be at its target)", (std::int32_t milliseconds))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.ForwardTime(std::chrono::milliseconds{ milliseconds });
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(fixture.target[joint].Value(), fixture.Joint(joint + 1).Position().Value(), positionTolerance) << "joint " << joint + 1;
}

THEN(R"(the move shall be aborted and the joints shall hold their positions for {int} ms)", (std::int32_t milliseconds))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.ForwardTime(controlPeriod);
    EXPECT_EQ(fixture.movesAbortedBefore + 1, fixture.events.movesAborted);
    EXPECT_FALSE(fixture.arm.Motion().Moving());

    const auto held = JointPositions(fixture);
    fixture.ForwardTime(std::chrono::milliseconds{ milliseconds });
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(held[joint].Value(), fixture.Joint(joint + 1).Position().Value(), positionTolerance) << "joint " << joint + 1;
}
