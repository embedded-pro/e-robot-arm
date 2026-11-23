#include "cucumber_cpp/Steps.hpp"
#include "integration_tests/support/ArmFixture.hpp"
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <string>

using integration::ArmFixture;
using namespace std::chrono_literals;

namespace
{
    constexpr float positionTolerance{ 0.005f };
    constexpr float jointTolerance{ 1.0e-3f };
    constexpr float torqueTolerance{ 0.05f };

    struct InverseAnswer
    {
        arm::IkResult result;
    };

    arm::JointPositions Measured(ArmFixture& fixture)
    {
        arm::JointPositions positions{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            positions[joint] = fixture.arm.Safety().Reading(joint).position;
        return positions;
    }

    arm::ArmModel Model(const ArmFixture& fixture)
    {
        return arm::ArmModel{ fixture.config.geometry, fixture.config.mass, fixture.config.positionLimits };
    }
}

THEN(R"(the tool pose of the measured joints shall be x {float}, y {float}, z {float} m)", (float x, float y, float z))
{
    auto& fixture = context.Get<ArmFixture>();
    const auto pose = arm::ToPlacement(Model(fixture).ToolPose(Measured(fixture)));
    std::cout << "[METRIC] model.tool_pose_m=" << pose.x << "," << pose.y << "," << pose.z << "\n";
    EXPECT_NEAR(x, pose.x, positionTolerance);
    EXPECT_NEAR(y, pose.y, positionTolerance);
    EXPECT_NEAR(z, pose.z, positionTolerance);
}

THEN(R"(the arm shall name the wrist and elbow singularities and not the shoulder)")
{
    auto& fixture = context.Get<ArmFixture>();
    const auto near = Model(fixture).Near(Measured(fixture));
    EXPECT_TRUE(near.wrist);
    EXPECT_TRUE(near.elbow);
    EXPECT_FALSE(near.shoulder);
}

THEN(R"(the arm shall name no singularity)")
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_FALSE(Model(fixture).Near(Measured(fixture)).Any());
}

WHEN(R"(the arm is asked for the joints that put the tool where it is now)")
{
    auto& fixture = context.Get<ArmFixture>();
    const auto model = Model(fixture);
    const auto measured = Measured(fixture);
    context.Emplace<InverseAnswer>(InverseAnswer{ model.Inverse(model.ToolPose(measured), measured) });
}

WHEN(R"(the arm is asked for the joints that put the tool at x {float}, y {float}, z {float} m, roll {float}, pitch {float}, yaw {float} rad)",
    (float x, float y, float z, float roll, float pitch, float yaw))
{
    auto& fixture = context.Get<ArmFixture>();
    context.Emplace<InverseAnswer>(InverseAnswer{ Model(fixture).Inverse(arm::ToTransform({ x, y, z, roll, pitch, yaw }), Measured(fixture)) });
}

WHEN(R"(the arm is asked for the joints that put the tool where joints {float}, {float}, {float}, {float}, {float}, {float} rad would)",
    (float q1, float q2, float q3, float q4, float q5, float q6))
{
    auto& fixture = context.Get<ArmFixture>();
    const auto model = Model(fixture);
    const arm::JointPositions joints{ foc::Radians{ q1 }, foc::Radians{ q2 }, foc::Radians{ q3 }, foc::Radians{ q4 }, foc::Radians{ q5 }, foc::Radians{ q6 } };
    context.Emplace<InverseAnswer>(InverseAnswer{ model.Inverse(model.ToolPose(joints), Measured(fixture)) });
}

THEN(R"(the answer shall be the measured joints)")
{
    auto& fixture = context.Get<ArmFixture>();
    const auto& answer = context.Get<InverseAnswer>().result;
    ASSERT_TRUE(answer.joints.has_value());
    const auto measured = Measured(fixture);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(measured[joint].Value(), (*answer.joints)[joint].Value(), jointTolerance) << "joint " << joint + 1;
}

THEN(R"(the arm shall refuse the pose as {string})", (const std::string& reason))
{
    const auto& answer = context.Get<InverseAnswer>().result;
    EXPECT_FALSE(answer.joints.has_value());
    EXPECT_EQ(reason == "unreachable" ? arm::IkFailure::unreachable : arm::IkFailure::outsideLimits, answer.failure);
}

THEN(R"(for {int} ms joint {int} shall hold with the gravity torque the arm reports for the measured joints)", (std::int32_t milliseconds, std::int32_t joint))
{
    auto& fixture = context.Get<ArmFixture>();
    const auto index = static_cast<std::size_t>(joint - 1);
    for (std::int32_t elapsed = 0; elapsed < milliseconds; elapsed += 10)
        fixture.ForwardTime(10ms);

    const auto gravity = Model(fixture).GravityTorque(Measured(fixture))[index].Value();
    const auto driving = fixture.Joint(joint).DrivingTorque();
    ASSERT_TRUE(driving.has_value());
    std::cout << "[METRIC] model.gravity_torque_nm=" << gravity << " driving=" << driving->Value() << " joint=" << joint << "\n";
    EXPECT_GT(std::abs(gravity), 0.1f);
    EXPECT_NEAR(gravity, driving->Value(), torqueTolerance);
}
