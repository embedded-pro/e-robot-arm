#include "core/collision/CollisionObserver.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    using namespace std::chrono_literals;

    constexpr auto period{ 10ms };
    constexpr float tolerance{ 2.0e-3f };

    arm::JointLimits WideLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        return limits;
    }

    class CollisionObserverTest
        : public testing::Test
    {
    public:
        CollisionObserverTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                positions[joint] = foc::Radians{ 0.1f * static_cast<float>(joint) };
            observer.Reset(positions, speeds);
        }

        arm::JointTorques Hold(const arm::JointTorques& extra, std::chrono::milliseconds duration)
        {
            arm::JointTorques residual{};
            for (auto elapsed = 0ms; elapsed < duration; elapsed += period)
            {
                auto commanded = model.GravityTorque(positions);
                for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                    commanded[joint] = foc::NewtonMeter{ commanded[joint].Value() + extra[joint].Value() };
                residual = observer.Update(positions, speeds, commanded, period);
            }
            return residual;
        }

        void ExpectResidual(const arm::JointTorques& expected, const arm::JointTorques& actual) const
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                EXPECT_NEAR(expected[joint].Value(), actual[joint].Value(), tolerance) << "joint " << joint;
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        arm::JointPositions positions{};
        arm::JointSpeeds speeds{};
        arm::DrivenArm drivenArm{ model, arm::JointInertias{ 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f } };
        arm::CollisionObserver observer{ drivenArm, 50.0f };
    };
}

TEST_F(CollisionObserverTest, sees_no_external_torque_on_an_arm_held_still_by_its_gravity_torque)
{
    ExpectResidual(arm::JointTorques{}, Hold(arm::JointTorques{}, 1s));
}

TEST_F(CollisionObserverTest, estimates_the_torque_of_an_obstacle_that_holds_a_joint_against_its_drive)
{
    arm::JointTorques pushed{};
    pushed[1] = foc::NewtonMeter{ 2.0f };

    arm::JointTorques obstacle{};
    obstacle[1] = foc::NewtonMeter{ -2.0f };
    ExpectResidual(obstacle, Hold(pushed, 1s));
}

TEST_F(CollisionObserverTest, filters_the_estimate_with_its_gain_as_a_first_order_lag)
{
    arm::JointTorques pushed{};
    pushed[1] = foc::NewtonMeter{ 2.0f };

    const auto residual = Hold(pushed, 20ms);

    EXPECT_NEAR(-2.0f * (1.0f - (1.0f - 0.5f) * (1.0f - 0.5f)), residual[1].Value(), 0.05f);
}

TEST_F(CollisionObserverTest, sees_no_external_torque_while_the_arm_moves_as_its_model_says)
{
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        speeds[joint] = foc::RadiansPerSecond{ 0.2f };
    observer.Reset(positions, speeds);

    arm::JointTorques residual{};
    for (auto elapsed = 0ms; elapsed < 500ms; elapsed += period)
    {
        const auto commanded = model.InverseDynamics(positions, speeds, arm::JointAccelerations{});
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            positions[joint] = foc::Radians{ positions[joint].Value() + speeds[joint].Value() * 0.01f };
        residual = observer.Update(positions, speeds, commanded, period);
    }

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(0.0f, residual[joint].Value(), 0.05f) << "joint " << joint;
}

TEST_F(CollisionObserverTest, starts_again_from_no_external_torque_after_a_reset)
{
    arm::JointTorques pushed{};
    pushed[1] = foc::NewtonMeter{ 2.0f };
    Hold(pushed, 1s);

    observer.Reset(positions, speeds);

    ExpectResidual(arm::JointTorques{}, Hold(arm::JointTorques{}, 1s));
}
