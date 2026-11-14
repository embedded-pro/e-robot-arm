#include "core/arm_controllers/implementations/SmoothedJointPosition.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    using namespace std::chrono_literals;

    constexpr float tolerance{ 1.0e-5f };

    class SmoothedJointPositionTest
        : public testing::Test
    {
    public:
        SmoothedJointPositionTest()
        {
            measured.positions.fill(foc::Radians{ 0.0f });
            reference.fill(foc::Radians{ 1.0f });
            controller.Reset(measured);
        }

        arm::SmoothedJointPosition controller{ 10ms, 40ms };
        arm::MeasuredJoints measured{};
        arm::JointPositions reference{};
    };
}

TEST_F(SmoothedJointPositionTest, starts_from_the_measured_joints_and_closes_a_fifth_of_the_gap_each_period)
{
    EXPECT_TRUE(controller.BumplessTransfer());

    const auto first = controller.Step(measured, reference);
    const auto second = controller.Step(measured, reference);

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_NEAR(0.2f, first.positions[joint].Value(), tolerance);
        EXPECT_NEAR(0.36f, second.positions[joint].Value(), tolerance);
    }
}

TEST_F(SmoothedJointPositionTest, converges_on_a_held_reference)
{
    arm::JointSetpoints setpoints{};
    for (int period = 0; period != 100; ++period)
        setpoints = controller.Step(measured, reference);

    EXPECT_NEAR(1.0f, setpoints.positions[3].Value(), 1.0e-4f);
}
