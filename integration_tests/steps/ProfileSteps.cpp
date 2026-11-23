#include "core/arm_profile/ArmProfile.hpp"
#include "cucumber_cpp/Steps.hpp"
#include "infra/stream/StdStringOutputStream.hpp"
#include "integration_tests/support/ArmFixture.hpp"
#include <gtest/gtest.h>
#include <optional>
#include <string>

using integration::ArmFixture;

namespace
{
    struct ProfileFile
    {
        std::string text;
        std::optional<arm::ArmProfile> read;
        std::string problem;
    };

    std::string Written(const arm::ArmProfile& profile)
    {
        infra::StdStringOutputStream::WithStorage stream;
        arm::WriteArmProfile(profile, stream);
        return stream.Storage();
    }

    ProfileFile& File(cucumber_cpp::library::Context& context)
    {
        return context.Get<ProfileFile>();
    }
}

GIVEN(R"(an arm profile whose joint {int} may move from {float} to {float} rad)", (std::int32_t joint, float lower, float upper))
{
    auto config = ArmFixture::DefaultConfig();
    config.positionLimits[static_cast<std::size_t>(joint - 1)] = { foc::Radians{ lower }, foc::Radians{ upper } };
    arm::ArmProfile profile{ {}, config };
    profile.name = "integration arm";
    context.Emplace<ProfileFile>(ProfileFile{ Written(profile), std::nullopt, {} });
}

GIVEN(R"(the profile's joint {int} gear ratio is changed to {float})", (std::int32_t joint, float ratio))
{
    auto& text = File(context).text;
    std::size_t at{ 0 };
    for (std::int32_t index = 0; index != joint; ++index)
        at = text.find("\"ratio\": ", at) + 1;
    const auto start = at - 1 + std::string{ "\"ratio\": " }.size();
    const auto end = text.find(',', start);
    text.replace(start, end - start, std::to_string(ratio));
}

WHEN(R"(the profile is read)")
{
    auto& file = File(context);
    infra::BoundedString::WithStorage<arm::profileProblemSize> problem;
    file.read = arm::ReadArmProfile(infra::BoundedConstString{ file.text.data(), file.text.size() }, problem);
    file.problem.assign(problem.begin(), problem.end());
}

THEN(R"(the profile shall be accepted and written back word for word)")
{
    const auto& file = File(context);
    ASSERT_TRUE(file.read.has_value()) << file.problem;
    EXPECT_EQ(file.text, Written(*file.read));
}

THEN(R"(the profile shall be refused, naming {string})", (const std::string& item))
{
    const auto& file = File(context);
    EXPECT_FALSE(file.read.has_value());
    EXPECT_THAT(file.problem, testing::HasSubstr(item));
}

WHEN(R"(an arm with six simulated joints is built from the profile read)")
{
    const auto& file = File(context);
    ASSERT_TRUE(file.read.has_value()) << file.problem;
    context.Emplace<ArmFixture>(file.read->config);
}
