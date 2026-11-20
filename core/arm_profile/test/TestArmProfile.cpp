#include "core/arm_profile/ArmProfile.hpp"
#include "infra/stream/StringOutputStream.hpp"
#include "gmock/gmock.h"
#include <chrono>
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

    const std::string geometry{ R"( "geometry": { "opwMetres": { "a1": 0.05, "a2": -0.01, "b": 0.02, "c1": 0.3, "c2": 0.4, "c3": 0.35, "c4": 0.08 },)"
                                R"( "jointSigns": [ 1, -1, 1, 1, -1, 1 ], "jointOffsetsRad": [ 0, 0.5, 0, 0, 0, -0.25 ],)"
                                R"( "tool": { "zM": 0.12, "pitchRad": 0.3 }, "base": { "xM": 1, "yawRad": 1.5 } },)" };

    std::string Link(const std::string& mass)
    {
        return R"({ "massKg": )" + mass + R"(, "centreM": { "z": 0.1 }, "inertiaKgM2": { "xx": 0.004, "yy": 0.004, "zz": 0.0005, "xy": 0.0001 } })";
    }

    std::string Mass(const std::string& firstLink, const std::string& payload = R"(, "payload": { "massKg": 0.75, "centreM": { "x": 0.02, "z": 0.04 } })")
    {
        return R"( "massProperties": { "links": [ )" + firstLink + ", " + Link("0.9") + ", " + Link("0.8") + ", " + Link("0.3") + ", " + Link("0.25") + ", " +
               Link("0.1") + " ]" + payload + " },";
    }

    const std::string configuredJoint{ Joint(1,
        R"(, "transmission": { "ratio": 50, "direction": "negative", "offsetRad": -0.125 },)"
        R"( "motorController": { "current": "slidingMode", "speed": "adrc", "position": "lqr", "gains": { "positionBandwidth": 25, "speedErrorWeight": 1.5 } },)"
        R"( "motor": { "preset": "teknic-m2310p-ln-04k", "inertiaGramCm2": 120.5 }, "trackingError": { "windowRad": 0.1, "timeS": 0.35 }, "collisionTorqueNm": 7.5)") };

    std::string WithFirstJerk(std::string text, const std::string& jerk)
    {
        const std::string acceleration{ R"("accelerationRadPerS2": 2.5)" };
        text.insert(text.find(acceleration) + acceleration.size(), R"(, "jerkRadPerS3": )" + jerk);
        return text;
    }

    std::size_t Count(const std::string& text, const std::string& part)
    {
        std::size_t count{ 0 };
        for (auto at = text.find(part); at != std::string::npos; at = text.find(part, at + part.size()))
            ++count;
        return count;
    }

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
    const auto profile = Read(Profile(configuredJoint, R"( "enableMode": "torque",)"));
    ASSERT_TRUE(profile.has_value()) << Problem();
    const auto& config = profile->config;

    EXPECT_EQ("bench \"arm\"", std::string(profile->name.begin(), profile->name.end()));
    EXPECT_EQ(std::chrono::milliseconds{ 5 }, config.controlPeriod);
    EXPECT_EQ(4, config.missedRequestsBeforeSilent);
    EXPECT_EQ(3, config.commandTimeoutPeriods);
    EXPECT_EQ(10, config.driveWatchdogPeriods);
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
    EXPECT_NEAR(arm::ReferenceGeometry().opw.c2, config.geometry.opw.c2, 1.0e-6f);
    EXPECT_EQ(arm::ReferenceMass().links[2].inertia, config.mass.links[2].inertia);
    EXPECT_EQ(0.0f, config.mass.payload.mass);
}

TEST_F(ArmProfileTest, reads_the_mass_properties_of_the_links_and_the_payload)
{
    const auto profile = Read(Profile(Joint(1), Mass(Link("1.25"))));
    ASSERT_TRUE(profile.has_value()) << Problem();
    const auto& mass = profile->config.mass;

    EXPECT_NEAR(1.25f, mass.links[0].mass, 1.0e-6f);
    EXPECT_NEAR(0.1f, mass.links[5].mass, 1.0e-6f);
    EXPECT_EQ((std::array<float, 3>{ 0.0f, 0.0f, 0.1f }), mass.links[3].centre);
    EXPECT_EQ((std::array<float, 6>{ 0.004f, 0.004f, 0.0005f, 0.0001f, 0.0f, 0.0f }), mass.links[1].inertia);
    EXPECT_NEAR(0.75f, mass.payload.mass, 1.0e-6f);
    EXPECT_EQ((std::array<float, 3>{ 0.02f, 0.0f, 0.04f }), mass.payload.centre);

    const auto withoutPayload = Read(Profile(Joint(1), Mass(Link("1"), "")));
    ASSERT_TRUE(withoutPayload.has_value()) << Problem();
    EXPECT_EQ(0.0f, withoutPayload->config.mass.payload.mass);
}

TEST_F(ArmProfileTest, reads_the_geometry_with_its_signs_offsets_tool_and_base)
{
    const auto profile = Read(Profile(Joint(1), geometry));
    ASSERT_TRUE(profile.has_value()) << Problem();
    const auto& read = profile->config.geometry;

    EXPECT_NEAR(0.05f, read.opw.a1, 1.0e-6f);
    EXPECT_NEAR(-0.01f, read.opw.a2, 1.0e-6f);
    EXPECT_NEAR(0.35f, read.opw.c3, 1.0e-6f);
    EXPECT_EQ(-1, read.opw.signs[1]);
    EXPECT_NEAR(-0.25f, read.opw.offsets[5], 1.0e-6f);
    EXPECT_NEAR(0.12f, read.tool.z, 1.0e-6f);
    EXPECT_NEAR(0.3f, read.tool.pitch, 1.0e-6f);
    EXPECT_NEAR(0.0f, read.tool.x, 1.0e-6f);
    EXPECT_NEAR(1.5f, read.base.yaw, 1.0e-6f);
}

TEST_F(ArmProfileTest, reads_the_link_meshes_and_leaves_the_others_out)
{
    const auto profile = Read(Profile(Joint(1), R"( "meshes": { "base": "meshes/base.glb", "link3": "forearm.gltf" },)"));
    ASSERT_TRUE(profile.has_value()) << Problem();

    EXPECT_EQ("meshes/base.glb", std::string(profile->meshes[0].begin(), profile->meshes[0].end()));
    EXPECT_EQ("forearm.gltf", std::string(profile->meshes[3].begin(), profile->meshes[3].end()));
    EXPECT_TRUE(profile->meshes[1].empty());
    EXPECT_TRUE(Read(Profile(Joint(1)))->meshes[6].empty());
}

TEST_F(ArmProfileTest, a_written_profile_reads_back_to_the_same_configuration)
{
    const auto original = Read(Profile(configuredJoint, geometry + Mass(Link("1.25")) + R"( "meshes": { "link6": "flange \"v2\".glb" },)" +
                                                            R"( "gravityPd": { "bandwidthRadPerS": 22.5, "dampingRatio": 0.9 },)" +
                                                            R"( "computedTorque": { "bandwidthRadPerS": 40, "dampingRatio": 1.2 },)" +
                                                            R"( "gravityPid": { "bandwidthRadPerS": 15, "dampingRatio": 0.8 },)" +
                                                            R"( "collisionObserverGainPerS": 40,)"));
    ASSERT_TRUE(original.has_value()) << Problem();

    infra::StringOutputStream::WithStorage<16384> stream;
    arm::WriteArmProfile(*original, stream);
    const auto& written = stream.Storage();
    const auto copy = Read(std::string{ written.begin(), written.end() });
    ASSERT_TRUE(copy.has_value()) << Problem() << "\n"
                                  << std::string{ written.begin(), written.end() };

    EXPECT_EQ(original->name, copy->name);
    EXPECT_EQ(original->config.controlPeriod, copy->config.controlPeriod);
    EXPECT_EQ(original->config.missedRequestsBeforeSilent, copy->config.missedRequestsBeforeSilent);
    EXPECT_EQ(original->config.gravityPd.bandwidth, copy->config.gravityPd.bandwidth);
    EXPECT_EQ(original->config.gravityPd.dampingRatio, copy->config.gravityPd.dampingRatio);
    EXPECT_EQ(original->config.computedTorque.bandwidth, copy->config.computedTorque.bandwidth);
    EXPECT_EQ(original->config.computedTorque.dampingRatio, copy->config.computedTorque.dampingRatio);
    EXPECT_EQ(original->config.gravityPid.bandwidth, copy->config.gravityPid.bandwidth);
    EXPECT_EQ(original->config.gravityPid.dampingRatio, copy->config.gravityPid.dampingRatio);
    EXPECT_EQ(original->config.collisionObserverGain, copy->config.collisionObserverGain);
    EXPECT_EQ(original->config.collision, copy->config.collision);
    EXPECT_EQ(original->config.nodes, copy->config.nodes);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_EQ(original->config.transmissions[joint].ratio, copy->config.transmissions[joint].ratio);
        EXPECT_EQ(original->config.transmissions[joint].direction, copy->config.transmissions[joint].direction);
        EXPECT_EQ(original->config.transmissions[joint].offset, copy->config.transmissions[joint].offset);
        EXPECT_EQ(original->config.positionLimits[joint].lower, copy->config.positionLimits[joint].lower);
        EXPECT_EQ(original->config.positionLimits[joint].upper, copy->config.positionLimits[joint].upper);
        EXPECT_EQ(original->config.motionLimits[joint].maxSpeed, copy->config.motionLimits[joint].maxSpeed);
        EXPECT_EQ(original->config.tracking[joint].window, copy->config.tracking[joint].window);
        EXPECT_EQ(original->config.tracking[joint].time, copy->config.tracking[joint].time);
        EXPECT_EQ(original->config.controllers[joint].has_value(), copy->config.controllers[joint].has_value());
    }

    const auto& before = *original->config.controllers[0];
    const auto& after = *copy->config.controllers[0];
    EXPECT_EQ(before.position, after.position);
    EXPECT_FALSE(arm::Compare(before, after).Any());

    ASSERT_TRUE(copy->config.motors[0].has_value());
    EXPECT_EQ(original->config.motors[0]->preset, copy->config.motors[0]->preset);
    EXPECT_FALSE(arm::Compare(original->config.motors[0]->settings, copy->config.motors[0]->settings).Any());

    const auto& geometryBefore = original->config.geometry;
    const auto& geometryAfter = copy->config.geometry;
    EXPECT_EQ(geometryBefore.opw.a2, geometryAfter.opw.a2);
    EXPECT_EQ(geometryBefore.opw.c4, geometryAfter.opw.c4);
    EXPECT_EQ(geometryBefore.opw.signs, geometryAfter.opw.signs);
    EXPECT_EQ(geometryBefore.opw.offsets, geometryAfter.opw.offsets);
    EXPECT_EQ(geometryBefore.tool.pitch, geometryAfter.tool.pitch);
    EXPECT_EQ(geometryBefore.base.x, geometryAfter.base.x);
    for (std::size_t link = 0; link != arm::jointCount; ++link)
    {
        EXPECT_EQ(original->config.mass.links[link].mass, copy->config.mass.links[link].mass);
        EXPECT_EQ(original->config.mass.links[link].centre, copy->config.mass.links[link].centre);
        EXPECT_EQ(original->config.mass.links[link].inertia, copy->config.mass.links[link].inertia);
    }
    EXPECT_EQ(original->config.mass.payload.mass, copy->config.mass.payload.mass);
    EXPECT_EQ(original->config.mass.payload.centre, copy->config.mass.payload.centre);
    EXPECT_EQ(original->meshes, copy->meshes);
    EXPECT_EQ("flange \"v2\".glb", std::string(copy->meshes[6].begin(), copy->meshes[6].end()));
    EXPECT_NE(std::string::npos, std::string(written.begin(), written.end()).find(R"("inertiaGramCm2": 120.5)"));
    EXPECT_NE(std::string::npos, std::string(written.begin(), written.end()).find(R"("fluxLinkageMilliWeber": 6.4)"));
}

TEST_F(ArmProfileTest, reads_the_following_error_supervision_and_leaves_it_off_when_it_is_left_out)
{
    const std::string custom{ R"("resistanceOhm": 0.36, "inductanceMilliHenry": 0.2, "polePairs": 4, "fluxLinkageMilliWeber": 6.4, "inertiaGramCm2": 70.6,)"
                              R"( "viscousFrictionMicroNmSPerRad": 15, "coulombFrictionMilliNm": 3.7, "continuousCurrentA": 7.1, "peakCurrentA": 20,)"
                              R"( "peakCurrentTimeS": 1, "overspeedLimitRadPerS": 754)" };
    const auto supervised = Read(Profile(Joint(1, R"(, "motor": { "preset": "teknic-m2310p-ln-04k", "followingErrorWindowRad": 2.5, "followingErrorTimeS": 0.05 })")));
    const auto unsupervised = Read(Profile(Joint(1, R"(, "motor": { )" + custom + " }")));

    ASSERT_TRUE(supervised.has_value()) << Problem();
    EXPECT_NEAR(2.5f, (*supervised->config.motors[0]).settings[arm::MotorParameter::followingErrorWindow], 1.0e-6f);
    EXPECT_NEAR(0.05f, (*supervised->config.motors[0]).settings[arm::MotorParameter::followingErrorTime], 1.0e-6f);
    ASSERT_TRUE(unsupervised.has_value()) << Problem();
    EXPECT_NEAR(0.0f, (*unsupervised->config.motors[0]).settings[arm::MotorParameter::followingErrorWindow], 1.0e-6f);
    EXPECT_NEAR(0.1f, (*unsupervised->config.motors[0]).settings[arm::MotorParameter::followingErrorTime], 1.0e-6f);
}

TEST_F(ArmProfileTest, reads_the_tracking_error_supervision_and_defaults_it_to_0_2_rad_for_0_2_s)
{
    const auto profile = Read(Profile(Joint(1, R"(, "trackingError": { "windowRad": 0.05, "timeS": 0.5 })")));

    ASSERT_TRUE(profile.has_value()) << Problem();
    EXPECT_NEAR(0.05f, profile->config.tracking[0].window.Value(), 1.0e-6f);
    EXPECT_EQ(std::chrono::milliseconds{ 500 }, profile->config.tracking[0].time);
    EXPECT_NEAR(0.2f, profile->config.tracking[1].window.Value(), 1.0e-6f);
    EXPECT_EQ(std::chrono::milliseconds{ 200 }, profile->config.tracking[1].time);
}

TEST_F(ArmProfileTest, reads_the_collision_supervision_and_defaults_it_to_20_n_m_with_a_gain_of_25_per_s)
{
    const auto tuned = Read(Profile(Joint(1, R"(, "collisionTorqueNm": 12.5)"), R"( "collisionObserverGainPerS": 80,)"));
    const auto plain = Read(Profile(Joint(1)));

    ASSERT_TRUE(tuned.has_value()) << Problem();
    EXPECT_NEAR(12.5f, tuned->config.collision[0].Value(), 1.0e-6f);
    EXPECT_NEAR(20.0f, tuned->config.collision[1].Value(), 1.0e-6f);
    EXPECT_NEAR(80.0f, tuned->config.collisionObserverGain, 1.0e-6f);
    ASSERT_TRUE(plain.has_value()) << Problem();
    EXPECT_NEAR(25.0f, plain->config.collisionObserverGain, 1.0e-6f);
}

TEST_F(ArmProfileTest, reads_the_controllers_bandwidth_and_damping_and_defaults_them_to_30_and_0_7)
{
    const auto tuned = Read(Profile(Joint(1), R"( "gravityPd": { "bandwidthRadPerS": 45, "dampingRatio": 1 }, "computedTorque": { "dampingRatio": 0.9 },)"));
    const auto plain = Read(Profile(Joint(1)));

    ASSERT_TRUE(tuned.has_value()) << Problem();
    EXPECT_NEAR(45.0f, tuned->config.gravityPd.bandwidth, 1.0e-6f);
    EXPECT_NEAR(1.0f, tuned->config.gravityPd.dampingRatio, 1.0e-6f);
    EXPECT_NEAR(30.0f, tuned->config.computedTorque.bandwidth, 1.0e-6f);
    EXPECT_NEAR(0.9f, tuned->config.computedTorque.dampingRatio, 1.0e-6f);
    EXPECT_NEAR(20.0f, tuned->config.gravityPid.bandwidth, 1.0e-6f);
    EXPECT_NEAR(0.7f, tuned->config.gravityPid.dampingRatio, 1.0e-6f);
    ASSERT_TRUE(plain.has_value()) << Problem();
    EXPECT_NEAR(30.0f, plain->config.gravityPd.bandwidth, 1.0e-6f);
    EXPECT_NEAR(0.7f, plain->config.gravityPd.dampingRatio, 1.0e-6f);
}

TEST_F(ArmProfileTest, reads_the_speed_trip_and_torque_limit_and_leaves_them_to_their_defaults_when_absent)
{
    const auto profile = Read(Profile(Joint(1, R"(, "speedLimitRadPerS": 2.5, "torqueLimitNm": 30)")));

    ASSERT_TRUE(profile.has_value()) << Problem();
    ASSERT_TRUE(profile->config.speedTrips[0].has_value());
    EXPECT_NEAR(2.5f, profile->config.speedTrips[0]->Value(), 1.0e-6f);
    ASSERT_TRUE(profile->config.torqueLimits[0].has_value());
    EXPECT_NEAR(30.0f, profile->config.torqueLimits[0]->Value(), 1.0e-6f);
    EXPECT_FALSE(profile->config.speedTrips[1].has_value());
    EXPECT_FALSE(profile->config.torqueLimits[1].has_value());

    infra::StringOutputStream::WithStorage<16384> stream;
    arm::WriteArmProfile(*profile, stream);
    const std::string written{ stream.Storage().begin(), stream.Storage().end() };
    EXPECT_EQ(1u, Count(written, R"("speedLimitRadPerS": 2.5)"));
    EXPECT_EQ(1u, Count(written, R"("torqueLimitNm": 30)"));
}

TEST_F(ArmProfileTest, reads_and_writes_the_drive_watchdog_in_control_periods)
{
    const auto profile = Read(Profile(Joint(1), R"( "driveWatchdogPeriods": 25,)"));
    ASSERT_TRUE(profile.has_value()) << Problem();
    EXPECT_EQ(25, profile->config.driveWatchdogPeriods);

    infra::StringOutputStream::WithStorage<16384> stream;
    arm::WriteArmProfile(*profile, stream);
    const std::string written{ stream.Storage().begin(), stream.Storage().end() };
    EXPECT_EQ(1u, Count(written, R"("driveWatchdogPeriods": 25,)"));
}

TEST_F(ArmProfileTest, reads_the_workspace_and_self_collision_and_writes_them_back)
{
    const std::string safety{ R"( "safety": { "workspaceM": { "min": { "x": -0.8, "y": -0.8, "z": 0.05 }, "max": { "x": 0.8, "y": 0.8, "z": 1 } },)"
                              R"( "forbiddenZonesM": [ { "min": { "x": 0.3, "y": -0.1, "z": 0 }, "max": { "x": 0.5, "y": 0.1, "z": 0.4 } } ],)"
                              R"( "linkRadiiM": [ 0.06, 0.05, 0.05, 0.04, 0.04, 0.03, 0.02 ], "selfCollisionMarginM": 0.02, "allowedLinkPairs": [ [ 3, 6 ] ] },)" };
    const auto profile = Read(Profile(Joint(1), safety));
    ASSERT_TRUE(profile.has_value()) << Problem();
    const auto& workspace = profile->config.workspace;

    ASSERT_TRUE(workspace.workspace.has_value());
    EXPECT_NEAR(0.05f, workspace.workspace->min[2], 1.0e-6f);
    EXPECT_NEAR(1.0f, workspace.workspace->max[2], 1.0e-6f);
    ASSERT_EQ(1, workspace.forbiddenZoneCount);
    EXPECT_NEAR(0.3f, workspace.forbiddenZones[0].min[0], 1.0e-6f);
    EXPECT_NEAR(0.06f, workspace.linkRadii[0], 1.0e-6f);
    EXPECT_NEAR(0.02f, workspace.linkRadii[6], 1.0e-6f);
    EXPECT_NEAR(0.02f, workspace.selfCollisionMargin, 1.0e-6f);
    ASSERT_EQ(1, workspace.allowedLinkPairCount);
    EXPECT_EQ((arm::LinkPair{ 2, 5 }), workspace.allowedLinkPairs[0]);

    infra::StringOutputStream::WithStorage<16384> stream;
    arm::WriteArmProfile(*profile, stream);
    const auto again = Read(std::string{ stream.Storage().begin(), stream.Storage().end() });
    ASSERT_TRUE(again.has_value()) << Problem();
    EXPECT_EQ(1, again->config.workspace.forbiddenZoneCount);
    EXPECT_EQ((arm::LinkPair{ 2, 5 }), again->config.workspace.allowedLinkPairs[0]);
    EXPECT_NEAR(0.05f, again->config.workspace.workspace->min[2], 1.0e-6f);
}

TEST_F(ArmProfileTest, a_profile_without_a_workspace_leaves_it_to_the_geometry_and_writes_none)
{
    const auto profile = Read(Profile(Joint(1)));
    ASSERT_TRUE(profile.has_value()) << Problem();
    EXPECT_FALSE(profile->config.workspace.workspace.has_value());

    infra::StringOutputStream::WithStorage<16384> stream;
    arm::WriteArmProfile(*profile, stream);
    const std::string written{ stream.Storage().begin(), stream.Storage().end() };
    EXPECT_EQ(0u, Count(written, "workspaceM"));
    EXPECT_EQ(1u, Count(written, R"("linkRadiiM": [ 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04 ])"));
}

TEST_F(ArmProfileTest, reads_a_version_1_profile_and_writes_version_7)
{
    const auto version1 = Read(Profile(Joint(1)));
    ASSERT_TRUE(version1.has_value()) << Problem();

    infra::StringOutputStream::WithStorage<16384> stream;
    arm::WriteArmProfile(*version1, stream);
    const std::string written{ stream.Storage().begin(), stream.Storage().end() };
    EXPECT_NE(std::string::npos, written.find(R"("version": 7,)"));

    auto version8{ Profile(Joint(1)) };
    version8.replace(version8.find(R"("version": 1)"), 12, R"("version": 8)");
    EXPECT_FALSE(Read(version8).has_value());
    EXPECT_EQ("version: is not a version this reader knows", Problem());
}

TEST_F(ArmProfileTest, reads_the_s_curve_and_each_joints_jerk_limit_and_writes_them_back)
{
    auto text = Profile(Joint(1), R"( "motionProfile": "s-curve",)");
    const std::string acceleration{ R"("accelerationRadPerS2": 2.5)" };
    for (auto at = text.find(acceleration); at != std::string::npos; at = text.find(acceleration, at + 1))
        text.insert(at + acceleration.size(), R"(, "jerkRadPerS3": 12.5)");

    const auto profile = Read(text);
    ASSERT_TRUE(profile.has_value()) << Problem();
    EXPECT_EQ(arm::MotionProfile::sCurve, profile->config.motionProfile);
    for (const auto& jerk : profile->config.jerkLimits)
    {
        ASSERT_TRUE(jerk.has_value());
        EXPECT_NEAR(12.5f, jerk->Value(), 1.0e-6f);
    }

    infra::StringOutputStream::WithStorage<16384> stream;
    arm::WriteArmProfile(*profile, stream);
    const std::string written{ stream.Storage().begin(), stream.Storage().end() };
    EXPECT_EQ(1u, Count(written, R"("motionProfile": "s-curve")"));
    EXPECT_EQ(6u, Count(written, R"("jerkRadPerS3": 12.5)"));

    const auto trapezoid = Read(Profile(Joint(1)));
    ASSERT_TRUE(trapezoid.has_value()) << Problem();
    EXPECT_EQ(arm::MotionProfile::trapezoid, trapezoid->config.motionProfile);
    EXPECT_FALSE(trapezoid->config.jerkLimits[0].has_value());
}

TEST_F(ArmProfileTest, reads_the_tool_limits_defaults_the_rest_and_writes_them_back)
{
    const auto defaults = Read(Profile(Joint(1)));
    ASSERT_TRUE(defaults.has_value()) << Problem();
    EXPECT_EQ(arm::ToolLimits{}, defaults->config.toolLimits);
    EXPECT_NEAR(0.25f, defaults->config.toolLimits.speedMPerS, 1.0e-6f);
    EXPECT_NEAR(0.001f, defaults->config.toolLimits.manipulabilityMin, 1.0e-6f);

    const auto profile = Read(Profile(Joint(1), R"( "toolLimits": { "speedMPerS": 0.5, "accelerationMPerS2": 2, "jerkMPerS3": 20, "angularSpeedRadPerS": 1.5, "angularAccelerationRadPerS2": 6, "angularJerkRadPerS3": 60, "manipulabilityMin": 0.01 },)"));
    ASSERT_TRUE(profile.has_value()) << Problem();
    const arm::ToolLimits expected{ 0.5f, 2.0f, 20.0f, 1.5f, 6.0f, 60.0f, 0.01f };
    EXPECT_EQ(expected, profile->config.toolLimits);

    infra::StringOutputStream::WithStorage<16384> stream;
    arm::WriteArmProfile(*profile, stream);
    const auto back = Read(std::string{ stream.Storage().begin(), stream.Storage().end() });
    ASSERT_TRUE(back.has_value()) << Problem();
    EXPECT_EQ(expected, back->config.toolLimits);
}

TEST_F(ArmProfileTest, reads_the_teach_points_and_writes_them_back)
{
    const auto none = Read(Profile(Joint(1)));
    ASSERT_TRUE(none.has_value()) << Problem();
    EXPECT_EQ(0u, none->config.teachPointCount);

    const auto profile = Read(Profile(Joint(1), R"( "teachPoints": [ { "name": "home", "jointsRad": [ 0, 0.5, -0.5, 0, 1, 0 ] }, { "name": "pick", "jointsRad": [ 0.25, 0, 0, 0, 0, -1 ] } ],)"));
    ASSERT_TRUE(profile.has_value()) << Problem();
    ASSERT_EQ(2u, profile->config.teachPointCount);
    EXPECT_EQ("home", (std::string{ profile->config.teachPoints[0].name.begin(), profile->config.teachPoints[0].name.end() }));
    EXPECT_NEAR(-0.5f, profile->config.teachPoints[0].joints[2].Value(), 1.0e-6f);
    EXPECT_NEAR(-1.0f, profile->config.teachPoints[1].joints[5].Value(), 1.0e-6f);

    infra::StringOutputStream::WithStorage<16384> stream;
    arm::WriteArmProfile(*profile, stream);
    const auto back = Read(std::string{ stream.Storage().begin(), stream.Storage().end() });
    ASSERT_TRUE(back.has_value()) << Problem();
    ASSERT_EQ(2u, back->config.teachPointCount);
    EXPECT_EQ("pick", (std::string{ back->config.teachPoints[1].name.begin(), back->config.teachPoints[1].name.end() }));
    EXPECT_NEAR(0.25f, back->config.teachPoints[1].joints[0].Value(), 1.0e-6f);
}

TEST_F(ArmProfileTest, reads_the_joint_impedance_stiffnesses_and_damping_ratio_and_writes_them_back)
{
    const auto defaults = Read(Profile(Joint(1)));
    ASSERT_TRUE(defaults.has_value()) << Problem();
    EXPECT_FALSE(defaults->config.jointImpedance.stiffness.has_value());
    EXPECT_NEAR(0.7f, defaults->config.jointImpedance.dampingRatio, 1.0e-6f);

    infra::StringOutputStream::WithStorage<16384> defaultStream;
    arm::WriteArmProfile(*defaults, defaultStream);
    const auto defaultBack = Read(std::string{ defaultStream.Storage().begin(), defaultStream.Storage().end() });
    ASSERT_TRUE(defaultBack.has_value()) << Problem();
    EXPECT_FALSE(defaultBack->config.jointImpedance.stiffness.has_value());

    const auto profile = Read(Profile(Joint(1), R"( "jointImpedance": { "stiffnessNmPerRad": [ 40, 80, 60, 10, 8, 4.5 ], "dampingRatio": 1.2 },)"));
    ASSERT_TRUE(profile.has_value()) << Problem();
    const arm::JointStiffnesses expected{ 40.0f, 80.0f, 60.0f, 10.0f, 8.0f, 4.5f };
    EXPECT_EQ(expected, profile->config.jointImpedance.stiffness);
    EXPECT_NEAR(1.2f, profile->config.jointImpedance.dampingRatio, 1.0e-6f);

    infra::StringOutputStream::WithStorage<16384> stream;
    arm::WriteArmProfile(*profile, stream);
    const auto back = Read(std::string{ stream.Storage().begin(), stream.Storage().end() });
    ASSERT_TRUE(back.has_value()) << Problem();
    EXPECT_EQ(expected, back->config.jointImpedance.stiffness);
    EXPECT_NEAR(1.2f, back->config.jointImpedance.dampingRatio, 1.0e-6f);
}

TEST_F(ArmProfileTest, refuses_a_profile_and_names_the_first_problem)
{
    const std::vector<std::pair<std::string, std::string>> cases{
        { Profile(Joint(1), R"( "colour": "red",)"), "colour: is not a profile item" },
        { Profile(Joint(1), R"( "motionProfile": "smooth",)"), "motionProfile: must be trapezoid or s-curve" },
        { WithFirstJerk(Profile(Joint(1)), "0"), "joints[0].motionLimits.jerkRadPerS3: must be above 0" },
        { Profile(Joint(1), R"( "motionProfile": "s-curve",)"), "motionProfile: s-curve needs a jerk limit on every joint" },
        { Profile(Joint(1, R"(, "transmission": { "ratio": 0 })")), "joints[0].transmission.ratio: must be above 0" },
        { Profile(Joint(4)), "joints[3].node: repeats the node of joints[0]" },
        { Profile(Joint(1), {}, 5), "joints: must list six joints" },
        { Profile(Joint(1), {}, 7), "joints: must list six joints" },
        { Profile(Joint(1, R"(, "motorController": { "current": "pid", "speed": "pid", "position": "pid", "gains": { "positionBandwidth": -1 } })")),
            "joints[0].motorController.gains.positionBandwidth: is outside the e-foc contract's range" },
        { Profile(Joint(1, R"(, "motorController": { "current": "pid", "speed": "fuzzy", "position": "pid" })")),
            "joints[0].motorController.speed: must be pid, lqi, adrc or twoDof" },
        { Profile(Joint(1, R"(, "motorController": { "current": "pid", "speed": "pid" })")), "joints[0].motorController.position: is missing" },
        { Profile(Joint(1), R"( "enableMode": "position",)"), "enableMode: must be torque: the arm runs its joints in torque mode only" },
        { Profile(Joint(1, R"(, "trackingError": { "windowRad": 0 })")), "joints[0].trackingError.windowRad: must be above 0" },
        { Profile(Joint(1), R"( "gravityPd": { "bandwidthRadPerS": 600 },)"), "gravityPd.bandwidthRadPerS: must be above 0 and at most 500" },
        { Profile(Joint(1), R"( "gravityPd": { "dampingRatio": 0 },)"), "gravityPd.dampingRatio: must be above 0 and at most 5" },
        { Profile(Joint(1, R"(, "trackingError": { "timeS": 11 })")), "joints[0].trackingError.timeS: must be above 0 and at most 10" },
        { Profile(Joint(1, R"(, "collisionTorqueNm": 0)")), "joints[0].collisionTorqueNm: must be above 0 and at most 1000" },
        { Profile(Joint(1), R"( "collisionObserverGainPerS": 2000,)"), "collisionObserverGainPerS: must be above 0 and at most 1000" },
        { Profile(Joint(1, R"(, "trackingError": { "gain": 1 })")), "joints[0].trackingError.gain: is not a profile item" },
        { Profile(Joint(1, R"(, "motor": { "preset": "maxon-ec45" })")),
            "joints[0].motor.preset: must be teknic-m2310p-ln-04k or anaheim-bly172s-24v-4000" },
        { Profile(Joint(1, R"(, "motor": { "resistanceOhm": 0.4 })")), "joints[0].motor.inductanceMilliHenry: is missing" },
        { Profile(Joint(1, R"(, "motor": { "preset": "teknic-m2310p-ln-04k", "polePairs": 4.5 })")), "joints[0].motor.polePairs: must be a whole number" },
        { Profile(Joint(1, R"(, "motor": { "preset": "teknic-m2310p-ln-04k", "resistanceOhm": 500 })")),
            "joints[0].motor.resistanceOhm: is outside the e-foc contract's range" },
        { Profile(Joint(1, R"(, "motor": { "preset": "teknic-m2310p-ln-04k", "torque": 1 })")), "joints[0].motor.torque: is not a profile item" },
        { Profile(Joint(1), R"( "geometry": { "jointSigns": [ 1, 1, 1, 1, 1, 1 ] },)"), "geometry.opwMetres: is missing" },
        { Profile(Joint(1), R"( "geometry": { "opwMetres": { "a1": 0, "a2": 0, "b": 0, "c1": 0, "c2": 0, "c3": 0.2, "c4": 0 } },)"),
            "geometry.opwMetres.c2: must be above 0" },
        { Profile(Joint(1), R"( "geometry": { "opwMetres": { "a1": 0, "a2": 0, "b": 0, "c1": 0, "c2": 0.2, "c3": 0.2 } },)"), "geometry.opwMetres.c4: is missing" },
        { Profile(Joint(1), R"( "geometry": { "opwMetres": { "a1": 0, "a2": 0, "b": 0, "c1": 0, "c2": 0.2, "c3": 0.2, "c4": 0 }, "jointSigns": [ 1, 2, 1, 1, 1, 1 ] },)"),
            "geometry.jointSigns[1]: must be 1 or -1" },
        { Profile(Joint(1), R"( "geometry": { "opwMetres": { "a1": 0, "a2": 0, "b": 0, "c1": 0, "c2": 0.2, "c3": 0.2, "c4": 0 }, "jointOffsetsRad": [ 0, 0 ] },)"),
            "geometry.jointOffsetsRad: must be a list of six" },
        { Profile(Joint(1), R"( "geometry": { "opwMetres": { "a1": 0, "a2": 0, "b": 0, "c1": 0, "c2": 0.2, "c3": 0.2, "c4": 0 }, "tool": { "wM": 1 } },)"),
            "geometry.tool.wM: is not a profile item" },
        { Profile(Joint(1), R"( "massProperties": { "payload": { "massKg": 1 } },)"), "massProperties.links: is missing" },
        { Profile(Joint(1), R"( "massProperties": { "links": [ ] },)"), "massProperties.links: must be a list of six" },
        { Profile(Joint(1), Mass(Link("-1"))), "massProperties.links[0].massKg: must be 0 or above" },
        { Profile(Joint(1), Mass(R"({ "centreM": { "x": 0 } })")), "massProperties.links[0].massKg: is missing" },
        { Profile(Joint(1), Mass(R"({ "massKg": 1, "centreM": { "w": 0 } })")), "massProperties.links[0].centreM.w: is not a profile item" },
        { Profile(Joint(1), Mass(R"({ "massKg": 1, "inertiaKgM2": { "xx": -0.1 } })")), "massProperties.links[0].inertiaKgM2: moments must be 0 or above" },
        { Profile(Joint(1), Mass(R"({ "massKg": 1, "inertiaKgM2": { "xx": 0.1, "yy": 0.1, "zz": 0.3 } })")),
            "massProperties.links[0].inertiaKgM2: each sum of two moments must be at least the third" },
        { Profile(Joint(1), Mass(Link("1"), R"(, "tip": 1)")), "massProperties.tip: is not a profile item" },
        { Profile(Joint(1), R"( "meshes": { "link7": "a.glb" },)"), "meshes.link7: is not a profile item" },
        { Profile(Joint(1), R"( "meshes": { "link1": "arm.stl" },)"), "meshes.link1: must name a .gltf or .glb file" },
        { Profile(Joint(1), R"( "meshes": { "base": "" },)"), "meshes.base: must be a path of 1 to 96 characters" },
        { Profile(Joint(1), R"( "meshes": { "base": 3 },)"), "meshes.base: must be a path of 1 to 96 characters" },
        { R"({ "format": "e-robot-arm profile", "version": 8 })", "version: is not a version this reader knows" },
        { Profile(Joint(1), R"( "jointImpedance": { "stiffnessNmPerRad": [ 1, 1, 1, 0, 1, 1 ] },)"), "jointImpedance.stiffnessNmPerRad[3]: must be above 0" },
        { Profile(Joint(1), R"( "jointImpedance": { "stiffnessNmPerRad": [ 1, 1, 1 ] },)"), "jointImpedance.stiffnessNmPerRad: must be a list of six" },
        { Profile(Joint(1), R"( "jointImpedance": { "dampingRatio": 6 },)"), "jointImpedance.dampingRatio: must be above 0 and at most 5" },
        { Profile(Joint(1), R"( "jointImpedance": { "mass": 1 },)"), "jointImpedance.mass: is not a profile item" },
        { Profile(Joint(1), R"( "teachPoints": [ { "name": "", "jointsRad": [ 0, 0, 0, 0, 0, 0 ] } ],)"), "teachPoints[0].name: must be text of 1 to 16 characters" },
        { Profile(Joint(1), R"( "teachPoints": [ { "name": "a", "jointsRad": [ 0, 0, 0 ] } ],)"), "teachPoints[0].jointsRad: must be a list of six angles" },
        { Profile(Joint(1), R"( "teachPoints": [ { "name": "a" } ],)"), "teachPoints[0]: needs a name and jointsRad" },
        { Profile(Joint(1), R"( "teachPoints": [ { "name": "a", "jointsRad": [ 0, 0, 0, 0, 0, 0 ] }, { "name": "a", "jointsRad": [ 1, 0, 0, 0, 0, 0 ] } ],)"), "teachPoints: names must be unique" },
        { Profile(Joint(1), R"( "toolLimits": { "speedMPerS": 0 },)"), "toolLimits.speedMPerS: must be above 0" },
        { Profile(Joint(1), R"( "toolLimits": { "manipulabilityMin": 2 },)"), "toolLimits.manipulabilityMin: must be 0 or above and at most 1" },
        { Profile(Joint(1), R"( "toolLimits": { "spin": 1 },)"), "toolLimits.spin: is not a profile item" },
        { Profile(Joint(1), R"( "driveWatchdogPeriods": 1,)"), "driveWatchdogPeriods: must be a whole number from 2 to 255" },
        { Profile(Joint(1), R"( "safety": { "workspaceM": { "min": { "x": 1 }, "max": { "x": 0.5, "y": 1, "z": 1 } } },)"), "safety.workspaceM: min must be below max on every axis" },
        { Profile(Joint(1), R"( "safety": { "linkRadiiM": [ 0.1, 0.1 ] },)"), "safety.linkRadiiM: must be a list of seven radii" },
        { Profile(Joint(1), R"( "safety": { "allowedLinkPairs": [ [ 2, 3 ] ] },)"), "safety.allowedLinkPairs[0]: must name two links that are not adjacent" },
        { Profile(Joint(1), R"( "safety": { "selfCollisionMarginM": 0.6 },)"), "safety.selfCollisionMarginM: must be from 0 to 0.5" },
        { Profile(Joint(1, R"(, "speedLimitRadPerS": 0)")), "joints[0].speedLimitRadPerS: must be above 0" },
        { Profile(Joint(1, R"(, "torqueLimitNm": 1001)")), "joints[0].torqueLimitNm: must be above 0 and at most 1000" },
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
