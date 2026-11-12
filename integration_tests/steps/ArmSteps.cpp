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

    arm::JointPositions Positions(float q1, float q2, float q3, float q4, float q5, float q6)
    {
        return { foc::Radians{ q1 }, foc::Radians{ q2 }, foc::Radians{ q3 }, foc::Radians{ q4 }, foc::Radians{ q5 }, foc::Radians{ q6 } };
    }

}

GIVEN(R"(an arm with six simulated joints on one CAN bus)")
{
    context.Emplace<ArmFixture>();
}

GIVEN(R"(the control loop is running)")
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.arm.Loop().Start();
    fixture.ExecuteAllActions();
}

GIVEN(R"(the joints are started in position mode)")
{
    context.Get<ArmFixture>().StartInPositionMode();
}

WHEN(R"(the arm is commanded to start the joints)")
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.lastAccepted = fixture.arm.Safety().Start();
}

THEN(R"(the command shall be refused)")
{
    EXPECT_FALSE(context.Get<ArmFixture>().lastAccepted);
}

WHEN(R"(the arm checks the contracts of its joints)")
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_TRUE(fixture.arm.Safety().QueryContracts());
    fixture.ForwardTime(20ms);
}

THEN(R"(the contracts shall be reported compatible)")
{
    const auto& contracts = context.Get<ArmFixture>().events.contracts;
    ASSERT_TRUE(contracts.has_value());
    EXPECT_EQ(can::ContractCompatibility::compatible, *contracts);
}

THEN(R"(the arm shall start every joint in position mode)")
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_TRUE(fixture.arm.Safety().SelectMode(can::FocMotorMode::position));
    fixture.ForwardTime(20ms);
    EXPECT_TRUE(fixture.arm.Safety().Start());
    fixture.ForwardTime(20ms);

    for (std::size_t joint = 1; joint <= arm::jointCount; ++joint)
    {
        EXPECT_EQ(can::FocMotorMode::position, fixture.Joint(joint).Mode()) << "joint " << joint;
        EXPECT_EQ(can::FocMotorState::running, fixture.Joint(joint).State()) << "joint " << joint;
    }
}

WHEN(R"(the arm moves the joints to {float}, {float}, {float}, {float}, {float}, {float} rad)", (float q1, float q2, float q3, float q4, float q5, float q6))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.target = Positions(q1, q2, q3, q4, q5, q6);
    fixture.lastAccepted = fixture.arm.Motion().MoveTo(fixture.target);
}

THEN(R"(halfway through the move every joint shall be between its start and its target)")
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_TRUE(fixture.lastAccepted);
    fixture.ForwardTime(500ms);

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        const auto position = fixture.Joint(joint + 1).Position().Value();
        const auto from = fixture.start[joint].Value();
        const auto to = fixture.target[joint].Value();
        const auto low = std::min(from, to);
        const auto high = std::max(from, to);
        EXPECT_GT(position, low) << "joint " << joint + 1;
        EXPECT_LT(position, high) << "joint " << joint + 1;
    }
}

THEN(R"(the move shall be done within {int} ms)", (std::int32_t milliseconds))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.ForwardTime(std::chrono::milliseconds{ milliseconds });
    EXPECT_EQ(1u, fixture.events.movesDone);
    EXPECT_FALSE(fixture.arm.Motion().Moving());
}

THEN(R"(every joint shall be at its target)")
{
    auto& fixture = context.Get<ArmFixture>();
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(fixture.target[joint].Value(), fixture.Joint(joint + 1).Position().Value(), positionTolerance) << "joint " << joint + 1;
}

THEN(R"(the move shall be refused and no joint shall move)")
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_FALSE(fixture.lastAccepted);
    fixture.ForwardTime(1s);

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(fixture.start[joint].Value(), fixture.Joint(joint + 1).Position().Value(), positionTolerance) << "joint " << joint + 1;
}

WHEN(R"(joint {int} faults)", (std::int32_t joint))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.Joint(static_cast<std::size_t>(joint)).Fault(can::FocFaultCode::overCurrent);
    fixture.ForwardTime(20ms);
}

WHEN(R"(joint {int} stops answering)", (std::int32_t joint))
{
    auto& fixture = context.Get<ArmFixture>();
    fixture.Joint(static_cast<std::size_t>(joint)).Silence();
    fixture.ForwardTime(50ms);
}

THEN(R"(the arm shall report joint {int} faulted)", (std::int32_t joint))
{
    const auto& faulted = context.Get<ArmFixture>().events.faultedJoints;
    EXPECT_EQ((std::vector<std::size_t>{ static_cast<std::size_t>(joint - 1) }), faulted);
}

THEN(R"(the arm shall report joint {int} silent)", (std::int32_t joint))
{
    const auto& silent = context.Get<ArmFixture>().events.silentJoints;
    EXPECT_EQ((std::vector<std::size_t>{ static_cast<std::size_t>(joint - 1) }), silent);
}

THEN(R"(every joint but joint {int} shall have been emergency stopped)", (std::int32_t excluded))
{
    auto& fixture = context.Get<ArmFixture>();
    for (std::size_t joint = 1; joint <= arm::jointCount; ++joint)
        EXPECT_EQ(static_cast<std::int32_t>(joint) == excluded ? 0u : 1u, fixture.Joint(joint).EmergencyStops()) << "joint " << joint;
}

THEN(R"(every joint shall have been emergency stopped)")
{
    auto& fixture = context.Get<ArmFixture>();
    for (std::size_t joint = 1; joint <= arm::jointCount; ++joint)
        EXPECT_EQ(1u, fixture.Joint(joint).EmergencyStops()) << "joint " << joint;
}

THEN(R"(the arm shall refuse to move)")
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_TRUE(fixture.arm.Safety().Tripped());
    EXPECT_FALSE(fixture.arm.Safety().Start());
}

WHEN(R"(the faults are cleared, the arm is re-armed and the joints are started again)")
{
    auto& fixture = context.Get<ArmFixture>();
    EXPECT_TRUE(fixture.arm.Safety().ClearFault());
    fixture.ForwardTime(20ms);
    EXPECT_TRUE(fixture.arm.Safety().Rearm());
    EXPECT_TRUE(fixture.arm.Safety().Start());
    fixture.ForwardTime(20ms);
}
