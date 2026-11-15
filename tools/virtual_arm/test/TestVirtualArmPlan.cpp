#include "tools/virtual_arm/VirtualArmPlan.hpp"
#include "gmock/gmock.h"
#include <chrono>

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
    EXPECT_EQ("teknic", plan[0].plant);
    EXPECT_EQ("anaheim", plan[1].plant);
    EXPECT_EQ("joint6.mem", plan[5].memoryFile);
    EXPECT_EQ(3u, plan[2].seed);
    EXPECT_TRUE(plan[0].hostsBus);
    EXPECT_FALSE(plan[1].hostsBus);

    EXPECT_EQ((std::vector<std::string>{ "--bus", "arm", "--node", "0", "--plant", "teknic", "--memory", "joint1.mem", "--calibrated", "--seed", "1",
                  "--plant-port", "40123", "--hub-port", "29537", "--host-bus" }),
        virtual_arm::EngineArguments(plan[0], "arm", 40123, 29537));
    EXPECT_EQ("--hub-port", virtual_arm::EngineArguments(plan[1], "arm", 40123, 29537).end()[-2]);
}

TEST_F(VirtualArmPlanTest, refuses_a_node_a_virtual_motor_cannot_answer_on_and_a_custom_motor)
{
    config.nodes[3] = 300;
    EXPECT_EQ("joint 4 has node 300: a virtual motor answers on nodes 1 to 256", std::get<std::string>(virtual_arm::PlanVirtualArm(config)));

    config.nodes[3] = 4;
    config.motors[2] = arm::JointMotor{ std::nullopt, arm::motorPresets[0].settings };
    EXPECT_EQ("joint 3 has a custom motor: a virtual motor takes e-foc's plant presets only", std::get<std::string>(virtual_arm::PlanVirtualArm(config)));
}
