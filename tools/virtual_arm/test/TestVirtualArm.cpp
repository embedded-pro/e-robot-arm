#include "tools/virtual_arm/VirtualArm.hpp"
#include "gmock/gmock.h"
#include <arpa/inet.h>
#include <cstdlib>
#include <format>
#include <fstream>
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

        std::filesystem::path directory{ std::filesystem::temp_directory_path() / "e-robot-arm-virtual-arm-test" };
        arm::ArmConfig config{ { 1, 2, 3, 4, 5, 6 }, 3, {}, {}, 10ms };
    };
}

TEST_F(VirtualArmTest, brings_up_six_motors_and_pushes_each_links_load_on_its_shaft)
{
    virtual_arm::VirtualArm arm{ config, Options() };
    ASSERT_EQ(std::nullopt, arm.Start());
    EXPECT_TRUE(arm.Up());

    const virtual_arm::ArmCoupling coupling{ config };
    virtual_arm::MotorStates states{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        states[joint] = { 0.1f * static_cast<float>(joint + 1), 0.0f };
    const auto expected = coupling.ShaftTorques(states);

    EXPECT_TRUE(Eventually([&]()
        {
            const auto torque = Recorded(directory / "joint2.mem.torque");
            return torque.has_value() && std::abs(*torque - expected[1]) < 1.0e-4f;
        }));
    EXPECT_GT(std::abs(expected[1]), 0.1f);

    EXPECT_EQ(std::nullopt, arm.Problem());
    arm.Collide(1, foc::NewtonMeter{ 2.0f });
    EXPECT_TRUE(Eventually([&]()
        {
            const auto torque = Recorded(directory / "joint2.mem.torque");
            return torque.has_value() && std::abs(*torque - (expected[1] + 2.0f)) < 1.0e-4f;
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
    setenv("FAKE_MOTOR_VERSION", "3", 1);
    virtual_arm::VirtualArm arm{ config, Options() };

    EXPECT_EQ("a motor's plant port failed: plant port protocol version 3, expected 2", arm.Start());
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

TEST_F(VirtualArmTest, a_missing_engine_does_not_come_up)
{
    auto options = Options();
    options.engine = "/nonexistent/e-foc-virtual-motor";
    virtual_arm::VirtualArm arm{ config, options };

    EXPECT_THAT(arm.Start().value_or(""), testing::StartsWith("joint "));
}
