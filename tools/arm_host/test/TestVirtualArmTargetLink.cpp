#include "can-lite/drivers/implementation/LoopbackSocket.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "tools/arm_host/VirtualArmTargetLink.hpp"
#include "tools/arm_host/test/ArmHostTestSupport.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <thread>
#include <vector>

namespace
{
    using namespace std::chrono_literals;
    using virtual_arm::MotorPhase;

    class VirtualArmTargetLinkTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        VirtualArmTargetLinkTest()
        {
            std::filesystem::remove_all(folder);
        }

        ~VirtualArmTargetLinkTest() override
        {
            std::filesystem::remove_all(folder);
        }

        std::unique_ptr<arm_host::VirtualArmTargetLink> Open(const std::string& engine)
        {
            return std::make_unique<arm_host::VirtualArmTargetLink>(arm_host::ArmTarget{ arm_host::TargetKind::virtualArm, "virtual:arm", engine }, config, 1000000,
                folder);
        }

        template<class Condition>
        bool Eventually(arm_host::VirtualArmTargetLink& link, Condition condition)
        {
            for (int attempt = 0; attempt != 2000; ++attempt)
            {
                link.Poll();
                ExecuteAllActions();
                if (condition())
                    return true;
                std::this_thread::sleep_for(2ms);
            }
            return false;
        }

        bool AllUp(const arm_host::VirtualArmTargetLink& link) const
        {
            return std::ranges::all_of(link.Status().motors, [](const virtual_arm::MotorReport& motor)
                {
                    return motor.phase == MotorPhase::up;
                });
        }

        std::filesystem::path folder{ std::filesystem::temp_directory_path() / "e-robot-arm-virtual-arm-link-test" };
        arm::ArmConfig config{ arm_host::test::Config() };
    };
}

TEST_F(VirtualArmTargetLinkTest, starts_six_motors_on_its_own_hub_and_restarts_one_joint)
{
    const auto link = Open(FAKE_VIRTUAL_MOTOR);
    ASSERT_TRUE(Eventually(*link, [&link]()
        {
            return link->Connected() || link->Failure().has_value();
        }));
    ASSERT_EQ(std::nullopt, link->Failure());
    EXPECT_EQ(link.get(), link->Motors());
    EXPECT_EQ(nullptr, link->Simulation());
    EXPECT_EQ(arm_host::VirtualArmTargetLink::connectTimeout, link->ConnectTimeout());
    EXPECT_TRUE(AllUp(*link));
    std::this_thread::sleep_for(100ms);
    EXPECT_TRUE(AllUp(*link));
    EXPECT_EQ(folder.string(), link->Status().folder);

    EXPECT_EQ(std::nullopt, link->Restart(2));
    EXPECT_TRUE(Eventually(*link, [&link]()
        {
            return link->Status().motors[2].phase == MotorPhase::up;
        }));

    auto changed = config;
    changed.motors[4] = arm::JointMotor{ 1, arm::motorPresets[1].settings };
    EXPECT_TRUE(link->Adopt(changed));
    EXPECT_TRUE(Eventually(*link, [&link]()
        {
            return link->Status().motors[4].plant == "anaheim" && link->Status().motors[4].phase == MotorPhase::up;
        }));

    changed.nodes[0] = 40;
    EXPECT_FALSE(link->Adopt(changed));
}

TEST_F(VirtualArmTargetLinkTest, fails_with_the_virtual_arms_reason_when_a_motor_does_not_come_up)
{
    const auto link = Open("/nonexistent/e-foc-virtual-motor");

    ASSERT_TRUE(Eventually(*link, [&link]()
        {
            return link->Failure().has_value();
        }));
    EXPECT_THAT(*link->Failure(), testing::StartsWith("joint "));
    EXPECT_FALSE(link->Connected());
}

TEST_F(VirtualArmTargetLinkTest, refuses_to_share_a_hub_port_that_another_bus_holds)
{
    const auto holder = services::loopback::Listen(services::loopback::virtualCanHubPort);
    ASSERT_NE(services::loopback::invalidSocket, holder);

    const auto link = Open(FAKE_VIRTUAL_MOTOR);
    EXPECT_EQ("the virtual CAN hub port 29536 is in use: another virtual arm or bus is running", link->Failure().value_or(""));
    EXPECT_FALSE(link->Connected());

    services::loopback::Close(holder);
}

TEST_F(VirtualArmTargetLinkTest, the_opener_puts_each_run_in_a_folder_only_the_user_can_open_and_leaves_other_targets_to_the_next_opener)
{
    setenv("XDG_RUNTIME_DIR", folder.c_str(), 1);
    std::vector<arm_host::TargetKind> delegated;
    const auto opener = arm_host::VirtualArmTargetOpener(
        [&delegated](const arm_host::ArmTarget& target, const arm::ArmConfig&) -> std::unique_ptr<arm_host::ArmTargetLink>
        {
            delegated.push_back(target.kind);
            return nullptr;
        },
        1000000);

    EXPECT_EQ(nullptr, opener({ arm_host::TargetKind::simulated, "" }, config));
    EXPECT_EQ(std::vector{ arm_host::TargetKind::simulated }, delegated);

    const auto link = opener({ arm_host::TargetKind::virtualArm, "virtual:arm", "/nonexistent/e-foc-virtual-motor" }, config);
    ASSERT_NE(nullptr, link);
    const auto status = link->Motors()->Status();
    EXPECT_EQ(folder / "e-robot-arm", std::filesystem::path{ status.folder }.parent_path());
    EXPECT_EQ(std::filesystem::perms::owner_all, std::filesystem::status(folder / "e-robot-arm").permissions());
    unsetenv("XDG_RUNTIME_DIR");
}
