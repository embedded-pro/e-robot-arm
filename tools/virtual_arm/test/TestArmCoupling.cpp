#include "tools/virtual_arm/ArmCoupling.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    using namespace std::chrono_literals;

    constexpr float controlPeriod{ 5.0e-5f };
    constexpr uint64_t millisecond{ 20 };

    arm::ArmConfig Geared()
    {
        arm::ArmConfig config{ { 1, 2, 3, 4, 5, 6 }, 3, {}, {}, 10ms };
        for (auto& limit : config.positionLimits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        config.transmissions[1] = { 50.0f, arm::JointDirection::negative, foc::Radians{ 0.2f } };
        config.transmissions[2] = { 30.0f, arm::JointDirection::positive, foc::Radians{ 0.0f } };
        return config;
    }

    class ArmCouplingTest
        : public testing::Test
    {
    public:
        virtual_arm::MotorSamples Holding() const
        {
            const arm::ArmModel model{ config.geometry, config.mass, config.positionLimits };
            const auto gravity = model.GravityTorque(joints);
            virtual_arm::MotorSamples motors{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                motors[joint] = virtual_arm::MotorSample{ config.transmissions[joint].MotorPosition(joints[joint]).Value(),
                    gravity[joint].Value() / config.transmissions[joint].Gain(), 100 + joint, controlPeriod };
            return motors;
        }

        virtual_arm::ShaftSpeeds EngageThenStep(virtual_arm::MotorSamples motors, const arm::JointTorques& collision = {})
        {
            coupling.Step(motors, collision);
            for (auto& motor : motors)
                if (motor.has_value())
                    motor->period += millisecond;
            return coupling.Step(motors, collision);
        }

        arm::ArmConfig config{ Geared() };
        arm::JointPositions joints{ foc::Radians{ 0.3f }, foc::Radians{ 0.6f }, foc::Radians{ -0.4f }, foc::Radians{ 0.1f }, foc::Radians{ 0.5f }, foc::Radians{ 0.0f } };
        virtual_arm::ArmCoupling coupling{ config };
    };
}

TEST_F(ArmCouplingTest, drives_no_shaft_until_every_motor_streams)
{
    auto motors = Holding();
    motors[4].reset();

    const auto shafts = coupling.Step(motors, {});

    EXPECT_FALSE(coupling.Engaged());
    for (const auto& shaft : shafts)
        EXPECT_FALSE(shaft.has_value());
}

TEST_F(ArmCouplingTest, motor_torques_that_carry_the_links_keep_every_shaft_still)
{
    const auto shafts = EngageThenStep(Holding());

    EXPECT_TRUE(coupling.Engaged());
    for (const auto& shaft : shafts)
    {
        ASSERT_TRUE(shaft.has_value());
        EXPECT_NEAR(0.0f, shaft->speed, 1.0e-3f);
    }
}

TEST_F(ArmCouplingTest, without_motor_torque_the_links_fall_and_drive_the_shafts_through_the_transmission)
{
    auto motors = Holding();
    for (auto& motor : motors)
        motor->torque = 0.0f;

    const auto shafts = EngageThenStep(motors);

    const arm::ArmModel model{ config.geometry, config.mass, config.positionLimits };
    ASSERT_TRUE(shafts[1].has_value());
    EXPECT_GT(std::abs(shafts[1]->speed), 1.0e-2f);
    EXPECT_LT(shafts[1]->speed * model.GravityTorque(joints)[1].Value() * config.transmissions[1].Gain(), 0.0f);
}

TEST_F(ArmCouplingTest, a_torque_on_one_joint_also_moves_the_joints_its_inertia_couples_it_to)
{
    auto motors = Holding();
    motors[1]->torque += 0.05f;

    const auto shafts = EngageThenStep(motors);

    EXPECT_GT(std::abs(shafts[1]->speed), 1.0e-2f);
    EXPECT_GT(std::abs(shafts[2]->speed), 1.0e-3f);
}

TEST_F(ArmCouplingTest, a_collision_pushes_its_joint)
{
    arm::JointTorques collision{};
    collision[2] = foc::NewtonMeter{ 4.0f };

    const auto shafts = EngageThenStep(Holding(), collision);

    EXPECT_GT(shafts[2]->speed, 1.0e-2f);
}

TEST_F(ArmCouplingTest, once_engaged_a_joint_without_its_motor_is_held_and_its_shaft_left_alone)
{
    coupling.Step(Holding(), {});
    auto motors = Holding();
    motors[3].reset();
    for (auto& motor : motors)
        if (motor.has_value())
            motor->torque = 0.0f;

    const auto shafts = EngageThenStep(motors);

    EXPECT_FALSE(shafts[3].has_value());
    EXPECT_TRUE(shafts[1].has_value());
}

TEST_F(ArmCouplingTest, each_drive_names_the_window_it_used_the_shafts_apparent_inertia_and_the_load_the_held_links_put_on_it)
{
    const auto motors = Holding();
    const auto shafts = coupling.Step(motors, {});

    const arm::ArmModel model{ config.geometry, config.mass, config.positionLimits };
    const auto diagonal = (model.JointMassMatrix(joints).at(1, 1) + 50.0f * 50.0f * arm::motorPresets[0].settings[arm::MotorParameter::inertia]) / (50.0f * 50.0f);
    ASSERT_TRUE(shafts[1].has_value());
    EXPECT_EQ(101u, shafts[1]->period);
    EXPECT_NEAR(motors[1]->torque, shafts[1]->torque, 1.0e-6f);
    EXPECT_GT(shafts[1]->inertia, 0.0f);
    EXPECT_LE(shafts[1]->inertia, diagonal * 1.0001f);
}

TEST_F(ArmCouplingTest, each_joint_advances_by_the_time_its_own_motor_ran_and_not_at_all_while_its_motor_stalls)
{
    auto motors = Holding();
    for (auto& motor : motors)
        motor->torque = 0.0f;
    coupling.Step(motors, {});

    motors[1]->period += millisecond;
    const auto afterOne = coupling.Step(motors, {});
    EXPECT_GT(std::abs(afterOne[1]->speed), 1.0e-2f);
    EXPECT_FLOAT_EQ(0.0f, afterOne[2]->speed);

    const auto stalled = coupling.Step(motors, {});
    EXPECT_FLOAT_EQ(afterOne[1]->speed, stalled[1]->speed);

    motors[1]->period += millisecond;
    const auto afterTwo = coupling.Step(motors, {});
    EXPECT_NEAR(2.0f * afterOne[1]->speed, afterTwo[1]->speed, 0.05f * std::abs(afterOne[1]->speed));
}

TEST_F(ArmCouplingTest, the_load_torque_is_what_the_motor_torque_leaves_after_accelerating_the_shaft_inertia)
{
    auto motors = Holding();
    for (auto& motor : motors)
        motor->torque = 0.0f;
    coupling.Step(motors, {});
    motors[1]->period += millisecond;

    const auto shafts = coupling.Step(motors, {});

    ASSERT_TRUE(shafts[1].has_value());
    const auto acceleration = shafts[1]->speed / (static_cast<float>(millisecond) * controlPeriod);
    EXPECT_NEAR(-shafts[1]->inertia * acceleration, shafts[1]->torque, 1.0e-3f * std::abs(shafts[1]->inertia * acceleration));
    EXPECT_GT(std::abs(shafts[1]->torque), 1.0e-4f);
}
