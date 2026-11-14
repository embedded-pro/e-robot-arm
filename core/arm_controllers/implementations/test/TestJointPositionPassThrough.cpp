#include "core/arm_controllers/implementations/JointPositionPassThrough.hpp"
#include "gmock/gmock.h"

namespace
{
    class JointPositionPassThroughTest
        : public testing::Test
    {
    public:
        arm::JointPositionPassThrough controller;
    };
}

TEST_F(JointPositionPassThroughTest, sends_the_reference_unchanged_and_is_not_bumpless)
{
    arm::MeasuredJoints measured{};
    measured.positions.fill(foc::Radians{ 0.1f });
    arm::JointPositions reference{};
    reference.fill(foc::Radians{ 0.7f });

    controller.Reset(measured);

    EXPECT_EQ(reference, controller.Step(measured, reference).positions);
    EXPECT_FALSE(controller.BumplessTransfer());
}
