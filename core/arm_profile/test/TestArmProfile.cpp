#include "core/arm_profile/ArmProfile.hpp"
#include "infra/stream/StringOutputStream.hpp"
#include "gmock/gmock.h"
#include <string>
#include <utility>
#include <vector>

namespace
{
    std::string Joint(int node, const std::string& extra = {})
    {
        return R"({ "node": )" + std::to_string(node) +
               R"(, "positionLimitsRad": { "lower": -1.5, "upper": 2.25 }, "motionLimits": { "speedRadPerS": 1, "accelerationRadPerS2": 2.5 })" + extra + " }";
    }

    std::string Profile(const std::string& firstJoint, const std::string& rootExtra = {}, int jointCount = 6)
    {
        std::string joints{ firstJoint };
        for (int node = 2; node <= jointCount; ++node)
            joints += ", " + Joint(node);

        return R"({ "format": "e-robot-arm profile", "version": 1, "name": "bench \"arm\"", "controlPeriodMs": 5,)"
               R"( "missedRequestsBeforeSilent": 4,)" +
               rootExtra + R"( "joints": [ )" + joints + " ] }";
    }

    const std::string configuredJoint{ Joint(1,
        R"(, "transmission": { "ratio": 50, "direction": "negative", "offsetRad": -0.125 },)"
        R"( "motorController": { "current": "slidingMode", "speed": "adrc", "position": "lqr", "gains": { "positionBandwidth": 25, "speedErrorWeight": 1.5 } },)"
        R"( "motor": { "preset": "teknic-m2310p-ln-04k", "inertiaGramCm2": 120.5 })") };

    class ArmProfileTest
        : public testing::Test
    {
    public:
        std::optional<arm::ArmProfile> Read(const std::string& text)
        {
            return arm::ReadArmProfile(infra::BoundedConstString{ text.data(), text.size() }, problem);
        }

        std::string Problem() const
        {
            return std::string{ problem.begin(), problem.end() };
        }

        infra::BoundedString::WithStorage<arm::profileProblemSize> problem;
    };
}

TEST_F(ArmProfileTest, reads_every_item_and_fills_in_the_defaults)
{
    const auto profile = Read(Profile(configuredJoint, R"( "enableMode": "speed",)"));
    ASSERT_TRUE(profile.has_value()) << Problem();
    const auto& config = profile->config;

    EXPECT_EQ("bench \"arm\"", std::string(profile->name.begin(), profile->name.end()));
    EXPECT_EQ(std::chrono::milliseconds{ 5 }, config.controlPeriod);
    EXPECT_EQ(4, config.missedRequestsBeforeSilent);
    EXPECT_EQ(3, config.commandTimeoutPeriods);
    EXPECT_EQ(can::FocMotorMode::speed, config.enableMode);
    EXPECT_EQ((arm::JointNodeIds{ 1, 2, 3, 4, 5, 6 }), config.nodes);

    EXPECT_NEAR(50.0f, config.transmissions[0].ratio, 1.0e-6f);
    EXPECT_EQ(arm::JointDirection::negative, config.transmissions[0].direction);
    EXPECT_NEAR(-0.125f, config.transmissions[0].offset.Value(), 1.0e-6f);
    EXPECT_NEAR(1.0f, config.transmissions[1].ratio, 1.0e-6f);
    EXPECT_NEAR(-1.5f, config.positionLimits[3].lower.Value(), 1.0e-6f);
    EXPECT_NEAR(2.25f, config.positionLimits[3].upper.Value(), 1.0e-6f);
    EXPECT_NEAR(2.5f, config.motionLimits[5].maxAcceleration.Value(), 1.0e-6f);

    ASSERT_TRUE(config.controllers[0].has_value());
    const auto& controller = *config.controllers[0];
    EXPECT_EQ(can::FocCurrentAlgorithm::slidingMode, controller.current);
    EXPECT_EQ(can::FocSpeedAlgorithm::adrc, controller.speed);
    EXPECT_EQ(can::FocPositionAlgorithm::lqr, controller.position);
    EXPECT_NEAR(25.0f, controller[arm::Gain::positionBandwidth], 1.0e-6f);
    EXPECT_NEAR(1.5f, controller[arm::Gain::speedErrorWeight], 1.0e-6f);
    EXPECT_NEAR(arm::ControllerSettings{}[arm::Gain::currentBandwidth], controller[arm::Gain::currentBandwidth], 1.0e-3f);
    EXPECT_FALSE(config.controllers[1].has_value());

    ASSERT_TRUE(config.motors[0].has_value());
    EXPECT_EQ(0u, config.motors[0]->preset);
    EXPECT_NEAR(1.205e-5f, config.motors[0]->settings[arm::MotorParameter::inertia], 1.0e-10f);
    EXPECT_NEAR(0.36f, config.motors[0]->settings[arm::MotorParameter::resistance], 1.0e-6f);
    EXPECT_NEAR(0.0064f, config.motors[0]->settings[arm::MotorParameter::fluxLinkage], 1.0e-7f);
    EXPECT_FALSE(config.motors[1].has_value());
}

TEST_F(ArmProfileTest, a_written_profile_reads_back_to_the_same_configuration)
{
    const auto original = Read(Profile(configuredJoint));
    ASSERT_TRUE(original.has_value()) << Problem();

    infra::StringOutputStream::WithStorage<8192> stream;
    arm::WriteArmProfile(*original, stream);
    const auto& written = stream.Storage();
    const auto copy = Read(std::string{ written.begin(), written.end() });
    ASSERT_TRUE(copy.has_value()) << Problem() << "\n"
                                  << std::string{ written.begin(), written.end() };

    EXPECT_EQ(original->name, copy->name);
    EXPECT_EQ(original->config.controlPeriod, copy->config.controlPeriod);
    EXPECT_EQ(original->config.missedRequestsBeforeSilent, copy->config.missedRequestsBeforeSilent);
    EXPECT_EQ(original->config.enableMode, copy->config.enableMode);
    EXPECT_EQ(original->config.nodes, copy->config.nodes);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_EQ(original->config.transmissions[joint].ratio, copy->config.transmissions[joint].ratio);
        EXPECT_EQ(original->config.transmissions[joint].direction, copy->config.transmissions[joint].direction);
        EXPECT_EQ(original->config.transmissions[joint].offset, copy->config.transmissions[joint].offset);
        EXPECT_EQ(original->config.positionLimits[joint].lower, copy->config.positionLimits[joint].lower);
        EXPECT_EQ(original->config.positionLimits[joint].upper, copy->config.positionLimits[joint].upper);
        EXPECT_EQ(original->config.motionLimits[joint].maxSpeed, copy->config.motionLimits[joint].maxSpeed);
        EXPECT_EQ(original->config.controllers[joint].has_value(), copy->config.controllers[joint].has_value());
    }

    const auto& before = *original->config.controllers[0];
    const auto& after = *copy->config.controllers[0];
    EXPECT_EQ(before.position, after.position);
    EXPECT_FALSE(arm::Compare(before, after).Any());

    ASSERT_TRUE(copy->config.motors[0].has_value());
    EXPECT_EQ(original->config.motors[0]->preset, copy->config.motors[0]->preset);
    EXPECT_FALSE(arm::Compare(original->config.motors[0]->settings, copy->config.motors[0]->settings).Any());
    EXPECT_NE(std::string::npos, std::string(written.begin(), written.end()).find(R"("inertiaGramCm2": 120.5)"));
    EXPECT_NE(std::string::npos, std::string(written.begin(), written.end()).find(R"("fluxLinkageMilliWeber": 6.4)"));
}

TEST_F(ArmProfileTest, refuses_a_profile_and_names_the_first_problem)
{
    const std::vector<std::pair<std::string, std::string>> cases{
        { Profile(Joint(1), R"( "colour": "red",)"), "colour: is not a profile item" },
        { Profile(Joint(1, R"(, "transmission": { "ratio": 0 })")), "joints[0].transmission.ratio: must be above 0" },
        { Profile(Joint(4)), "joints[3].node: repeats the node of joints[0]" },
        { Profile(Joint(1), {}, 5), "joints: must list six joints" },
        { Profile(Joint(1), {}, 7), "joints: must list six joints" },
        { Profile(Joint(1, R"(, "motorController": { "current": "pid", "speed": "pid", "position": "pid", "gains": { "positionBandwidth": -1 } })")),
            "joints[0].motorController.gains.positionBandwidth: is outside the e-foc contract's range" },
        { Profile(Joint(1, R"(, "motorController": { "current": "pid", "speed": "fuzzy", "position": "pid" })")),
            "joints[0].motorController.speed: must be pid, lqi, adrc or twoDof" },
        { Profile(Joint(1, R"(, "motorController": { "current": "pid", "speed": "pid" })")), "joints[0].motorController.position: is missing" },
        { Profile(Joint(1, R"(, "motor": { "preset": "maxon-ec45" })")),
            "joints[0].motor.preset: must be teknic-m2310p-ln-04k or anaheim-bly172s-24v-4000" },
        { Profile(Joint(1, R"(, "motor": { "resistanceOhm": 0.4 })")), "joints[0].motor.inductanceMilliHenry: is missing" },
        { Profile(Joint(1, R"(, "motor": { "preset": "teknic-m2310p-ln-04k", "polePairs": 4.5 })")), "joints[0].motor.polePairs: must be a whole number" },
        { Profile(Joint(1, R"(, "motor": { "preset": "teknic-m2310p-ln-04k", "resistanceOhm": 500 })")),
            "joints[0].motor.resistanceOhm: is outside the e-foc contract's range" },
        { Profile(Joint(1, R"(, "motor": { "preset": "teknic-m2310p-ln-04k", "torque": 1 })")), "joints[0].motor.torque: is not a profile item" },
        { R"({ "format": "e-robot-arm profile", "version": 2 })", "version: is not a version this reader knows" },
        { R"({ "format": "something else" })", "format: must be \"e-robot-arm profile\"" },
        { R"({ "format": "e-robot-arm profile", "version": 1, "controlPeriodMs": 5 })", "name: is missing" },
        { R"({ "format": "e-robot-arm profile", "version": 1, "controlPeriodMs": 0 })", "controlPeriodMs: must be a whole number from 1 to 1000" },
        { R"({ "format": "e-robot-arm profile", "version": 1, "name": "x" "joints": [] })", "profile: is not valid JSON" },
    };

    for (const auto& [text, expected] : cases)
    {
        EXPECT_FALSE(Read(text).has_value()) << text;
        EXPECT_EQ(expected, Problem()) << text;
    }

    auto reversed = Profile(Joint(1));
    reversed.replace(reversed.find("-1.5"), 4, "3.00");
    EXPECT_FALSE(Read(reversed).has_value());
    EXPECT_EQ("joints[0].positionLimitsRad.lower: must be below upper", Problem());
}
