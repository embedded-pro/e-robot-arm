#include "cucumber_cpp/Steps.hpp"
#include "integration_tests/support/ArmFixture.hpp"
#include <algorithm>
#include <chrono>
#include <gtest/gtest.h>

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
