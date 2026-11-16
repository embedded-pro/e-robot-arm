#include "tools/virtual_arm/VirtualArmPlan.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <format>

namespace
{
    using namespace std::chrono_literals;

    class VirtualArmPlanTest
        : public testing::Test
    {
    public:
        arm::ArmConfig config{ { 1, 2, 3, 4, 5, 6 }, 3, {}, {}, 10ms };
    };
}

TEST_F(VirtualArmPlanTest, starts_each_joint_at_its_board_id_with_its_plant_and_memory_and_the_first_hosts_the_bus)
{
    config.motors[1] = arm::JointMotor{ 1, arm::motorPresets[1].settings };
    const auto plan = std::get<virtual_arm::VirtualArmPlan>(virtual_arm::PlanVirtualArm(config));

    EXPECT_EQ(0u, plan[0].boardId);
    EXPECT_EQ(5u, plan[5].boardId);
    EXPECT_EQ("teknic", plan[0].plant.name);
    EXPECT_EQ("anaheim", plan[1].plant.name);
    EXPECT_EQ("joint6.plant", plan[5].plantFile);
    EXPECT_EQ("joint6.mem", plan[5].memoryFile);
    EXPECT_EQ(3u, plan[2].seed);
    EXPECT_TRUE(plan[0].hostsBus);
    EXPECT_FALSE(plan[1].hostsBus);

    EXPECT_EQ((std::vector<std::string>{ "--bus", "arm", "--node", "0", "--plant", "joint1.plant", "--memory", "joint1.mem", "--calibrated", "--seed", "1",
                  "--plant-port", "40123", "--hub-port", "29537", "--host-bus" }),
        virtual_arm::EngineArguments(plan[0], "arm", 40123, 29537));
    EXPECT_EQ("--hub-port", virtual_arm::EngineArguments(plan[1], "arm", 40123, 29537).end()[-2]);
}

TEST_F(VirtualArmPlanTest, refuses_a_node_a_virtual_motor_cannot_answer_on)
{
    config.nodes[3] = 300;
    EXPECT_EQ("joint 4 has node 300: a virtual motor answers on nodes 1 to 256", std::get<std::string>(virtual_arm::PlanVirtualArm(config)));
}

TEST_F(VirtualArmPlanTest, a_joint_without_a_motor_is_the_teknic_preset_carrying_its_links)
{
    const auto plan = std::get<virtual_arm::VirtualArmPlan>(virtual_arm::PlanVirtualArm(config));

    EXPECT_EQ(std::format("e-foc plant description 1\npreset teknic\nload-inertia {}\n", virtual_arm::LinkInertias(config)[0]),
        plan[0].plant.description);
}

TEST_F(VirtualArmPlanTest, a_preset_motor_carries_only_the_values_the_profile_changed)
{
    auto settings = arm::motorPresets[1].settings;
    settings[arm::MotorParameter::resistance] = 0.5f;

    const auto plant = virtual_arm::PlantOf(arm::JointMotor{ 1, settings }, 0.25f);

    EXPECT_EQ("anaheim", plant.name);
    EXPECT_EQ("e-foc plant description 1\npreset anaheim\nresistance 0.5\nload-inertia 0.25\n", plant.description);
}

TEST_F(VirtualArmPlanTest, a_custom_motor_is_the_teknic_preset_with_every_value_that_differs_in_si_units)
{
    auto settings = arm::motorPresets[0].settings;
    settings[arm::MotorParameter::inductance] = 0.5f;
    settings[arm::MotorParameter::polePairs] = 7.0f;
    settings[arm::MotorParameter::peakCurrent] = 15.0f;
    settings[arm::MotorParameter::followingErrorWindow] = 2.0f;

    const auto plant = virtual_arm::PlantOf(arm::JointMotor{ std::nullopt, settings }, 0.0f);

    EXPECT_EQ("custom", plant.name);
    EXPECT_EQ("e-foc plant description 1\npreset teknic\ninductance 5e-04\npole-pairs 7\nmax-current 15\nload-inertia 0\n", plant.description);
}

TEST_F(VirtualArmPlanTest, each_motor_carries_its_links_at_home_through_its_gear)
{
    const auto direct = virtual_arm::LinkInertias(config);
    config.transmissions[1] = { 50.0f, arm::JointDirection::negative };
    const auto geared = virtual_arm::LinkInertias(config);

    for (const auto inertia : direct)
        EXPECT_GT(inertia, 0.0f);
    EXPECT_NEAR(direct[1] / 2500.0f, geared[1], direct[1] * 1e-6f);
    EXPECT_EQ(direct[2], geared[2]);
}
