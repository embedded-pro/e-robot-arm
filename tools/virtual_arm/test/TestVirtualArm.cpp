#include "can-lite/drivers/implementation/LoopbackSocket.hpp"
#include "tools/virtual_arm/VirtualArm.hpp"
#include "gmock/gmock.h"
#include <arpa/inet.h>
#include <cstdlib>
#include <format>
#include <fstream>
#include <iterator>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace
{
    using namespace std::chrono_literals;

    std::optional<float> Recorded(const std::filesystem::path& file)
    {
        std::ifstream stream{ file };
        float value{ 0.0f };
        if (stream >> value)
            return value;
        return std::nullopt;
    }

    std::string Text(const std::filesystem::path& file)
    {
        std::ifstream stream{ file };
        std::string text;
        std::getline(stream, text);
        return text;
    }

    std::string Content(const std::filesystem::path& file)
    {
        std::ifstream stream{ file };
        return { std::istreambuf_iterator<char>{ stream }, std::istreambuf_iterator<char>{} };
    }

    uint16_t FreeLoopbackPort()
    {
        const auto probe = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        bind(probe, static_cast<const sockaddr*>(static_cast<const void*>(&address)), sizeof(address));
        socklen_t length{ sizeof(address) };
        getsockname(probe, static_cast<sockaddr*>(static_cast<void*>(&address)), &length);
        close(probe);
        return ntohs(address.sin_port);
    }

    template<class Condition>
    bool Eventually(Condition condition)
    {
        for (int attempt = 0; attempt != 200; ++attempt)
        {
            if (condition())
                return true;
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }

    class VirtualArmTest
        : public testing::Test
    {
    public:
        VirtualArmTest()
        {
            unsetenv("FAKE_MOTOR_VERSION");
            unsetenv("FAKE_MOTOR_EXIT_AFTER_MS");
            std::filesystem::remove_all(directory);
            for (auto& limit : config.positionLimits)
                limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        }

        ~VirtualArmTest() override
        {
            std::filesystem::remove_all(directory);
        }

        virtual_arm::VirtualArmOptions Options() const
        {
            virtual_arm::VirtualArmOptions options{ FAKE_VIRTUAL_MOTOR, "arm", directory, 3s };
            options.hubPort = FreeLoopbackPort();
            return options;
        }

        virtual_arm::VirtualArmOptions JoiningOptions()
        {
            auto options = Options();
            options.hostBus = false;
            hub = services::loopback::Listen(options.hubPort);
            return options;
        }

        bool EventuallyIn(const virtual_arm::VirtualArm& arm, std::size_t joint, virtual_arm::MotorPhase phase)
        {
            return Eventually([&arm, joint, phase]()
                {
                    return arm.Motors()[joint].phase == phase;
                });
        }

        void TearDown() override
        {
            services::loopback::Close(hub);
        }

        std::filesystem::path directory{ std::filesystem::temp_directory_path() / "e-robot-arm-virtual-arm-test" };
        arm::ArmConfig config{ { 1, 2, 3, 4, 5, 6 }, 3, {}, {}, 10ms };
        intptr_t hub{ services::loopback::invalidSocket };
        const arm::JointMotor anaheim{ 1, arm::motorPresets[1].settings };
    };
}

TEST_F(VirtualArmTest, brings_up_six_motors_and_drives_each_shaft_with_the_arms_dynamics)
{
    virtual_arm::VirtualArm arm{ config, Options() };
    ASSERT_EQ(std::nullopt, arm.Start());
    EXPECT_TRUE(arm.Up());

    virtual_arm::ArmCoupling coupling{ config };
    virtual_arm::MotorSamples samples{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        samples[joint] = virtual_arm::MotorSample{ 0.1f * static_cast<float>(joint + 1), 0.0f, 0 };
    const auto falling = coupling.Step(samples, {}, 1.0e-3f)[1]->speed;
    ASSERT_GT(std::abs(falling), 1.0e-3f);

    EXPECT_TRUE(Eventually([&]()
        {
            const auto shaft = Recorded(directory / "joint2.mem.shaft");
            return shaft.has_value() && *shaft * falling > 0.0f;
        }));

    EXPECT_EQ(std::nullopt, arm.Problem());
    arm.Collide(1, foc::NewtonMeter{ falling > 0.0f ? -500.0f : 500.0f });
    EXPECT_TRUE(Eventually([&]()
        {
            const auto shaft = Recorded(directory / "joint2.mem.shaft");
            return shaft.has_value() && *shaft * falling < 0.0f;
        }));

    arm.Silence(4, true);
    EXPECT_TRUE(Eventually([this]()
        {
            return std::filesystem::exists(directory / "joint5.mem.paused");
        }));
    EXPECT_EQ(std::nullopt, arm.Problem());
    arm.Silence(4, false);
    EXPECT_TRUE(Eventually([this]()
        {
            return !std::filesystem::exists(directory / "joint5.mem.paused");
        }));

    arm.Stop();
    EXPECT_FALSE(arm.Up());
}

TEST_F(VirtualArmTest, refuses_motors_of_another_protocol_version_and_names_the_reason)
{
    setenv("FAKE_MOTOR_VERSION", "2", 1);
    virtual_arm::VirtualArm arm{ config, Options() };

    EXPECT_EQ("a motor's plant port failed: plant port protocol version 2, expected 3", arm.Start());
    EXPECT_FALSE(arm.Up());
}

TEST_F(VirtualArmTest, reports_a_motor_that_ends_unasked)
{
    setenv("FAKE_MOTOR_EXIT_AFTER_MS", "300", 1);
    virtual_arm::VirtualArm arm{ config, Options() };
    ASSERT_EQ(std::nullopt, arm.Start());

    EXPECT_TRUE(Eventually([&arm]()
        {
            return arm.Problem().has_value();
        }));
    EXPECT_THAT(arm.Problem().value_or(""), testing::MatchesRegex("joint [1-6]'s motor ended unasked, exit code 3"));
    EXPECT_TRUE(EventuallyIn(arm, 5, virtual_arm::MotorPhase::ended));
    EXPECT_EQ(3, arm.Motors()[5].exitCode);
}

TEST_F(VirtualArmTest, refuses_to_start_while_the_hub_port_is_held_and_no_hub_answers_on_it)
{
    const auto holder = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(0, bind(holder, static_cast<const sockaddr*>(static_cast<const void*>(&address)), sizeof(address)));
    socklen_t length{ sizeof(address) };
    getsockname(holder, static_cast<sockaddr*>(static_cast<void*>(&address)), &length);

    auto options = Options();
    options.hubPort = ntohs(address.sin_port);
    options.hubPortTimeout = 300ms;
    virtual_arm::VirtualArm arm{ config, options };

    EXPECT_EQ(std::format("the virtual CAN hub port {} is still held by a closed connection", options.hubPort), arm.Start().value_or(""));
    close(holder);
}

TEST_F(VirtualArmTest, an_engine_named_relative_to_the_current_directory_starts_from_the_working_directory)
{
    auto options = Options();
    options.engine = std::filesystem::path{ "." } / std::filesystem::relative(FAKE_VIRTUAL_MOTOR);
    ASSERT_TRUE(options.engine.has_parent_path());
    virtual_arm::VirtualArm arm{ config, options };

    EXPECT_EQ(std::nullopt, arm.Start());
}

TEST_F(VirtualArmTest, a_missing_engine_does_not_come_up)
{
    auto options = Options();
    options.engine = "/nonexistent/e-foc-virtual-motor";
    virtual_arm::VirtualArm arm{ config, options };

    EXPECT_THAT(arm.Start().value_or(""), testing::StartsWith("joint "));
}

TEST_F(VirtualArmTest, reports_each_motor_up_with_its_plant_and_stopped_once_stopped)
{
    config.motors[3] = anaheim;
    virtual_arm::VirtualArm arm{ config, Options() };
    EXPECT_EQ(virtual_arm::MotorPhase::stopped, arm.Motors()[0].phase);
    ASSERT_EQ(std::nullopt, arm.Start());

    for (const auto& motor : arm.Motors())
        EXPECT_EQ(virtual_arm::MotorPhase::up, motor.phase);
    EXPECT_EQ("teknic", arm.Motors()[0].plant);
    EXPECT_EQ("anaheim", arm.Motors()[3].plant);

    arm.Stop();
    for (const auto& motor : arm.Motors())
        EXPECT_EQ(virtual_arm::MotorPhase::stopped, motor.phase);
}

TEST_F(VirtualArmTest, joins_a_listening_hub_with_no_motor_hosting_the_bus)
{
    virtual_arm::VirtualArm arm{ config, JoiningOptions() };
    ASSERT_EQ(std::nullopt, arm.Start());

    for (std::size_t joint = 1; joint <= arm::jointCount; ++joint)
        EXPECT_FALSE(std::filesystem::exists(directory / std::format("joint{}.mem.host", joint)));
}

TEST_F(VirtualArmTest, refuses_to_join_a_hub_that_does_not_listen)
{
    auto options = Options();
    options.hostBus = false;
    virtual_arm::VirtualArm arm{ config, options };

    EXPECT_EQ(std::format("no virtual CAN hub listens on port {}", options.hubPort), arm.Start().value_or(""));
}

TEST_F(VirtualArmTest, restarts_a_joint_with_another_motor_on_a_fresh_memory)
{
    virtual_arm::VirtualArm arm{ config, JoiningOptions() };
    ASSERT_EQ(std::nullopt, arm.Start());
    std::ofstream{ directory / "joint3.mem" } << "calibration";

    EXPECT_EQ(std::nullopt, arm.Restart(2, anaheim));

    EXPECT_TRUE(Eventually([&arm, this]()
        {
            return arm.Motors()[2].phase == virtual_arm::MotorPhase::up && arm.Motors()[2].plant == "anaheim";
        }));
    EXPECT_EQ("joint3.plant", Text(directory / "joint3.mem.plant"));
    EXPECT_EQ(virtual_arm::PlantOf(anaheim, virtual_arm::LinkInertias(config)[2]).description, Content(directory / "joint3.plant"));
    EXPECT_FALSE(std::filesystem::exists(directory / "joint3.mem"));
    EXPECT_EQ(std::nullopt, arm.Problem());
}

TEST_F(VirtualArmTest, restarts_a_joint_with_a_custom_motor)
{
    virtual_arm::VirtualArm arm{ config, JoiningOptions() };
    ASSERT_EQ(std::nullopt, arm.Start());

    EXPECT_EQ(std::nullopt, arm.Restart(1, arm::JointMotor{ std::nullopt, arm::motorPresets[0].settings }));

    EXPECT_TRUE(Eventually([&arm]()
        {
            return arm.Motors()[1].phase == virtual_arm::MotorPhase::up && arm.Motors()[1].plant == "custom";
        }));
    EXPECT_EQ(std::nullopt, arm.Problem());
}

TEST_F(VirtualArmTest, powers_a_joint_off_and_restarts_it_on_its_own_memory)
{
    virtual_arm::VirtualArm arm{ config, JoiningOptions() };
    ASSERT_EQ(std::nullopt, arm.Start());
    std::ofstream{ directory / "joint4.mem" } << "calibration";

    EXPECT_EQ(std::nullopt, arm.PowerOff(3));
    EXPECT_TRUE(EventuallyIn(arm, 3, virtual_arm::MotorPhase::stopped));

    EXPECT_EQ(std::nullopt, arm.Restart(3, std::nullopt));
    EXPECT_TRUE(EventuallyIn(arm, 3, virtual_arm::MotorPhase::up));
    EXPECT_EQ("calibration", Text(directory / "joint4.mem"));
    EXPECT_EQ(std::nullopt, arm.Problem());
}

TEST_F(VirtualArmTest, refuses_to_restart_a_joint_and_names_the_reason)
{
    virtual_arm::VirtualArm arm{ config, Options() };
    EXPECT_EQ("the virtual arm is not up", arm.Restart(1, std::nullopt).value_or(""));
    ASSERT_EQ(std::nullopt, arm.Start());

    EXPECT_EQ("joint 1's motor hosts the virtual CAN bus: restart the whole virtual arm", arm.Restart(0, std::nullopt).value_or(""));
    EXPECT_EQ("joint 1's motor hosts the virtual CAN bus: restart the whole virtual arm", arm.PowerOff(0).value_or(""));
    EXPECT_EQ("no joint 7", arm.Restart(6, std::nullopt).value_or(""));
}

TEST_F(VirtualArmTest, adopts_a_profile_restarting_only_the_joints_whose_motor_changed)
{
    virtual_arm::VirtualArm arm{ config, JoiningOptions() };
    ASSERT_EQ(std::nullopt, arm.Start());
    std::ofstream{ directory / "joint1.mem" } << "calibration";

    auto changed = config;
    changed.motors[4] = anaheim;
    EXPECT_EQ(std::nullopt, arm.Adopt(changed));

    EXPECT_TRUE(Eventually([&arm]()
        {
            return arm.Motors()[4].phase == virtual_arm::MotorPhase::up && arm.Motors()[4].plant == "anaheim";
        }));
    EXPECT_EQ("calibration", Text(directory / "joint1.mem"));
    EXPECT_EQ("teknic", arm.Motors()[0].plant);
}

TEST_F(VirtualArmTest, refuses_a_profile_it_cannot_adopt_and_names_the_reason)
{
    virtual_arm::VirtualArm arm{ config, Options() };
    EXPECT_EQ("the virtual arm is not up", arm.Adopt(config).value_or(""));
    ASSERT_EQ(std::nullopt, arm.Start());

    auto renumbered = config;
    renumbered.nodes[2] = 9;
    EXPECT_EQ("the profile's node ids changed: restart the whole virtual arm", arm.Adopt(renumbered).value_or(""));

    auto hostChanged = config;
    hostChanged.motors[0] = anaheim;
    EXPECT_EQ("joint 1's motor hosts the virtual CAN bus: restart the whole virtual arm", arm.Adopt(hostChanged).value_or(""));
}
