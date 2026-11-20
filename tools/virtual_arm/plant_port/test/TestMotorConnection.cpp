#include "tools/virtual_arm/plant_port/MotorConnection.hpp"
#include "tools/virtual_arm/plant_port/test/PlantPortBytes.hpp"
#include "gmock/gmock.h"
#include <numbers>

namespace
{
    using namespace virtual_arm::plant_port;
    using test::Bytes;

    constexpr float tolerance{ 1.0e-5f };

    class MotorConnectionTest
        : public testing::Test
    {
    public:
        void Receive(const std::vector<uint8_t>& bytes)
        {
            connection.Receive(bytes);
        }

        MotorConnection connection;
    };
}

TEST_F(MotorConnectionTest, accepts_a_version_3_hello_and_fails_on_another)
{
    Receive(test::HelloFrame());
    ASSERT_TRUE(connection.Introduction().has_value());
    EXPECT_EQ("teknic", connection.Introduction()->plant);
    EXPECT_FALSE(connection.Failure().has_value());

    MotorConnection other;
    other.Receive(test::HelloFrame(2));
    EXPECT_FALSE(other.Introduction().has_value());
    EXPECT_EQ("plant port protocol version 2, expected 3", other.Failure());
}

TEST_F(MotorConnectionTest, fails_on_a_frame_before_hello_or_a_malformed_frame)
{
    Receive(test::WindowFrame(0.0f, 0.0f));
    EXPECT_EQ("frame 0x10 before hello", connection.Failure());

    MotorConnection other;
    other.Receive(test::HelloFrame());
    other.Receive(Bytes{}.U8(1).Frame(MessageType::status));
    EXPECT_EQ("malformed frame 0x02", other.Failure());
}

TEST_F(MotorConnectionTest, follows_the_rotor_across_turns_and_its_electrical_torque)
{
    Receive(test::HelloFrame());
    EXPECT_FALSE(connection.RotorAngle().has_value());
    EXPECT_FALSE(connection.ElectricalTorque().has_value());

    Receive(test::WindowFrame(6.2f, 3.0f));
    Receive(test::WindowFrame(0.05f, 3.5f, 0.25f));
    ASSERT_TRUE(connection.RotorAngle().has_value());
    EXPECT_NEAR(0.25f, *connection.ElectricalTorque(), tolerance);
    EXPECT_EQ(std::optional<uint64_t>{ 149 }, connection.LastPeriod());
    EXPECT_NEAR(0.05f + 2.0f * std::numbers::pi_v<float>, *connection.RotorAngle(), tolerance);
    EXPECT_NEAR(3.5f, *connection.RotorSpeed(), tolerance);

    Receive(test::WindowFrame(6.2f, -3.0f));
    EXPECT_NEAR(6.2f, *connection.RotorAngle(), 1.0e-4f);
}

TEST_F(MotorConnectionTest, keeps_status_refusal_and_the_newest_log_lines_and_skips_unknown_frames)
{
    Receive(test::HelloFrame());
    Receive(Bytes{}.U64(5).F32(0.99f).U32(2).U8(0).U8(1).U8(0).Frame(MessageType::status));
    Receive(Bytes{}.U8(0x85).U8(0).Frame(MessageType::refused));
    Receive(EncodeFrame(static_cast<MessageType>(0x13), std::vector<uint8_t>{ 1, 2, 3 }));
    for (int line = 0; line != 21; ++line)
        Receive(Bytes{}.Text("line " + std::to_string(line)).Frame(MessageType::logLine));

    ASSERT_TRUE(connection.LastStatus().has_value());
    EXPECT_EQ(2u, connection.LastStatus()->overruns);
    ASSERT_TRUE(connection.LastRefusal().has_value());
    EXPECT_EQ(0x85u, connection.LastRefusal()->request);
    ASSERT_EQ(keptLogLines, connection.RecentLog().size());
    EXPECT_EQ("line 1", connection.RecentLog().front());
    EXPECT_EQ("line 20", connection.RecentLog().back());
    EXPECT_FALSE(connection.Failure().has_value());
}

TEST_F(MotorConnectionTest, queues_requests_in_order_until_taken)
{
    connection.Stream(true);
    connection.ShaftSpeed(ShaftDrive{ 1.5f, 7, 0.1f, 2.0e-4f });
    connection.Pause();
    EXPECT_TRUE(connection.Paused());
    connection.Resume();
    EXPECT_FALSE(connection.Paused());
    connection.ClearAll();

    auto expected = EncodeStream(true);
    for (const auto& frame : { EncodeShaftSpeed(ShaftDrive{ 1.5f, 7, 0.1f, 2.0e-4f }), EncodeFrame(MessageType::pause), EncodeFrame(MessageType::resume), EncodeFrame(MessageType::clearAll) })
        expected.insert(expected.end(), frame.begin(), frame.end());
    EXPECT_EQ(expected, connection.TakeOutgoing());
    EXPECT_TRUE(connection.TakeOutgoing().empty());
}
