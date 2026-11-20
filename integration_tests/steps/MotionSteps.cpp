#include "cucumber_cpp/Steps.hpp"
#include "integration_tests/support/ArmFixture.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>

using integration::ArmFixture;
using namespace std::chrono_literals;

namespace
{
    arm::JointPositions Readings(ArmFixture& fixture)
    {
        arm::JointPositions positions{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            positions[joint] = fixture.arm.Safety().Reading(joint).position;
        return positions;
    }

    arm::ArmModel Model()
    {
        auto limits = ArmFixture::DefaultConfig().positionLimits;
        return arm::ArmModel{ arm::ReferenceGeometry(), arm::ReferenceMass(), limits };
    }

    float LargestDifference(const arm::JointPositions& a, const arm::JointPositions& b)
    {
        float largest{ 0.0f };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            largest = std::max(largest, std::abs(a[joint].Value() - b[joint].Value()));
        return largest;
    }

    float DistanceFromLine(const math::Vector3<float>& point, const math::Vector3<float>& from, const math::Vector3<float>& to)
    {
        const auto direction = to - from;
        const auto offset = point - from;
        const auto length2 = direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2];
        const auto along = std::clamp((offset[0] * direction[0] + offset[1] * direction[1] + offset[2] * direction[2]) / length2, 0.0f, 1.0f);
        const auto nearest = from + direction * along;
        const auto d = point - nearest;
        return std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    }

    bool Stopped(const ArmFixture& fixture)
    {
        const auto state = fixture.arm.Lifecycle().State();
        return state == arm::ArmState::tripped || state == arm::ArmState::holding;
    }

    void CommandLine(ArmFixture& fixture, const arm::ToolPose& target)
    {
        fixture.start = Readings(fixture);
        fixture.lineFrom = Model().ToolPose(fixture.start);
        fixture.lineTo = target;
        fixture.movesDoneBefore = fixture.events.movesDone;
        fixture.lastAccepted = fixture.arm.Lifecycle().MoveLine(target);
    }
}

GIVEN(R"(an arm with six simulated joints, jerk limits and slow tool limits)")
{
    auto config = ArmFixture::DefaultConfig();
    config.jerkLimits.fill(foc::RadiansPerSecondCubed{ 20.0f });
    config.toolLimits = { 0.05f, 0.2f, 2.0f, 0.2f, 0.8f, 8.0f, 0.001f };
    context.Emplace<ArmFixture>(config);
}

GIVEN(R"(the arm moves with the S-curve profile)")
{
    EXPECT_TRUE(context.Get<ArmFixture>().arm.Motion().SelectProfile(arm::MotionProfile::sCurve));
}

WHEN(R"(after {int} ms the arm moves the joints to {float}, {float}, {float}, {float}, {float}, {float} rad)", (std::int32_t milliseconds, float q1, float q2, float q3, float q4, float q5, float q6))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.ForwardTime(std::chrono::milliseconds{ milliseconds });
    fixture.target = { foc::Radians{ q1 }, foc::Radians{ q2 }, foc::Radians{ q3 }, foc::Radians{ q4 }, foc::Radians{ q5 }, foc::Radians{ q6 } };
    fixture.movesDoneBefore = fixture.events.movesDone;
    fixture.lastAccepted = fixture.arm.Lifecycle().MoveTo(fixture.target);
    EXPECT_TRUE(fixture.lastAccepted);
}

GIVEN(R"(a waypoint at {float}, {float}, {float}, {float}, {float}, {float} rad)", (float q1, float q2, float q3, float q4, float q5, float q6))
{
    context.Get<ArmFixture>().waypoints.push_back({ foc::Radians{ q1 }, foc::Radians{ q2 }, foc::Radians{ q3 }, foc::Radians{ q4 }, foc::Radians{ q5 }, foc::Radians{ q6 } });
}

WHEN(R"(the arm moves through the waypoints)")
{
    auto& fixture = context.Get<ArmFixture>();
    arm::JointWaypoints::WithMaxSize<arm::maxWaypoints> waypoints;
    for (const auto& waypoint : fixture.waypoints)
        waypoints.push_back(waypoint);
    fixture.target = fixture.waypoints.back();
    fixture.movesDoneBefore = fixture.events.movesDone;
    fixture.lastAccepted = fixture.arm.Lifecycle().MoveThrough(waypoints);
}

THEN(R"(the path shall be done within {int} ms, passing within {float} rad of every waypoint)", (std::int32_t milliseconds, float tolerance))
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_TRUE(fixture.lastAccepted);
    std::vector<float> closest(fixture.waypoints.size(), 1.0e9f);
    for (std::int32_t elapsed = 0; elapsed < milliseconds && fixture.arm.Motion().Moving(); elapsed += 10)
    {
        fixture.ForwardTime(10ms);
        const auto positions = Readings(fixture);
        for (std::size_t waypoint = 0; waypoint != fixture.waypoints.size(); ++waypoint)
            closest[waypoint] = std::min(closest[waypoint], LargestDifference(positions, fixture.waypoints[waypoint]));
    }

    EXPECT_EQ(fixture.movesDoneBefore + 1, fixture.events.movesDone);
    EXPECT_FALSE(Stopped(fixture));
    for (std::size_t waypoint = 0; waypoint != fixture.waypoints.size(); ++waypoint)
        EXPECT_LT(closest[waypoint], tolerance) << "waypoint " << waypoint + 1;
}

WHEN(R"(the arm moves the tool in a straight line by {float}, {float}, {float} m)", (float x, float y, float z))
{
    auto& fixture = context.Get<ArmFixture>();
    auto target = Model().ToolPose(Readings(fixture));
    target.translation[0] += x;
    target.translation[1] += y;
    target.translation[2] += z;
    CommandLine(fixture, target);
}

WHEN(R"(the arm moves the tool in a straight line to the point {string})", (const std::string& name))
{
    auto& fixture = context.Get<ArmFixture>();
    CommandLine(fixture, Model().ToolPose(fixture.points.at(name)));
}

THEN(R"(the line shall be refused, no move shall start and every joint shall hold within {float} rad)", (float tolerance))
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_FALSE(fixture.lastAccepted);
    ASSERT_TRUE(fixture.arm.Motion().LineRefused().has_value());
    for (int period = 0; period != 100; ++period)
    {
        fixture.ForwardTime(10ms);
        EXPECT_FALSE(fixture.arm.Motion().Moving());
        EXPECT_LT(LargestDifference(Readings(fixture), fixture.start), tolerance);
    }
    EXPECT_EQ(fixture.movesDoneBefore, fixture.events.movesDone);
}

THEN(R"(the line shall be done within {int} ms with the tool within {float} m of the line and at rest within {float} m of its target)",
    (std::int32_t milliseconds, float tolerance, float endTolerance))
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_TRUE(fixture.lastAccepted);
    const auto model = Model();
    float farthest{ 0.0f };
    for (std::int32_t elapsed = 0; elapsed < milliseconds && fixture.arm.Motion().Moving(); elapsed += 10)
    {
        fixture.ForwardTime(10ms);
        farthest = std::max(farthest, DistanceFromLine(model.ToolPose(Readings(fixture)).translation, fixture.lineFrom.translation, fixture.lineTo.translation));
    }

    EXPECT_EQ(fixture.movesDoneBefore + 1, fixture.events.movesDone);
    EXPECT_FALSE(Stopped(fixture));
    EXPECT_LT(farthest, tolerance);
    for (int period = 0; period != 100; ++period)
        fixture.ForwardTime(10ms);
    const auto end = model.ToolPose(Readings(fixture)).translation - fixture.lineTo.translation;
    EXPECT_LT(std::sqrt(end[0] * end[0] + end[1] * end[1] + end[2] * end[2]), endTolerance);
}

WHEN(R"(the arm jogs joint {int} at {float} rad\/s for {int} ms)", (std::int32_t joint, float speed, std::int32_t milliseconds))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.movesDoneBefore = fixture.events.movesDone;
    const arm::JointJog jog{ static_cast<std::size_t>(joint - 1), foc::RadiansPerSecond{ speed } };
    for (std::int32_t elapsed = 0; elapsed < milliseconds; elapsed += 10)
    {
        EXPECT_TRUE(fixture.arm.Lifecycle().Jog(jog));
        fixture.ForwardTime(10ms);
    }
}

WHEN(R"(the arm jogs the tool along the base y axis at {float} m\/s for {int} ms)", (float speed, std::int32_t milliseconds))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.movesDoneBefore = fixture.events.movesDone;
    const arm::ToolJog jog{ arm::JogFrame::base, arm::ToolAxis::y, speed };
    for (std::int32_t elapsed = 0; elapsed < milliseconds; elapsed += 10)
    {
        EXPECT_TRUE(fixture.arm.Lifecycle().Jog(jog));
        fixture.ForwardTime(10ms);
    }
}

THEN(R"(within {int} ms the jog shall be done with every joint at rest)", (std::int32_t milliseconds))
{
    auto& fixture = context.Get<ArmFixture>();
    for (std::int32_t elapsed = 0; elapsed < milliseconds && fixture.arm.Motion().Moving(); elapsed += 10)
        fixture.ForwardTime(10ms);
    for (int period = 0; period != 30; ++period)
        fixture.ForwardTime(10ms);

    EXPECT_FALSE(fixture.arm.Motion().Moving());
    EXPECT_EQ(fixture.movesDoneBefore + 1, fixture.events.movesDone);
    EXPECT_FALSE(Stopped(fixture));
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_LT(std::abs(fixture.arm.Safety().Reading(joint).speed.Value()), 0.05f) << "joint " << joint + 1;
}

WHEN(R"(the arm teaches the point {string})", (const std::string& name))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.points[name] = Readings(fixture);
}

WHEN(R"(the arm goes to the point {string})", (const std::string& name))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.target = fixture.points.at(name);
    fixture.movesDoneBefore = fixture.events.movesDone;
    fixture.lastAccepted = fixture.arm.Lifecycle().MoveTo(fixture.target);
}
