#include "hal/interfaces/test_doubles/CanMock.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "tools/arm_host/BusLoadMeter.hpp"
#include "gmock/gmock.h"

namespace
{
    using namespace std::chrono_literals;
    using testing::_;
    using testing::SaveArg;

    class BusLoadMeterTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        BusLoadMeterTest()
        {
            EXPECT_CALL(can, ReceiveData(_)).WillOnce(SaveArg<0>(&received));
            meter.emplace(can, 1000000);
            meter->ReceiveData([this](hal::Can::Id, const hal::Can::Message&)
                {
                    ++delivered;
                });
        }

        static hal::Can::Message Bytes(std::size_t count)
        {
            hal::Can::Message message;
            for (std::size_t byte = 0; byte != count; ++byte)
                message.push_back(static_cast<uint8_t>(byte));
            return message;
        }

        testing::StrictMock<hal::CanMock> can;
        infra::Function<void(hal::Can::Id id, const hal::Can::Message& data)> received;
        std::optional<arm_host::BusLoadMeter> meter;
        uint32_t delivered{ 0 };
    };
}

TEST_F(BusLoadMeterTest, reports_nothing_before_a_whole_second_has_passed)
{
    ForwardTime(999ms);

    EXPECT_FALSE(meter->Load().has_value());
}

TEST_F(BusLoadMeterTest, counts_sent_and_received_frames_over_the_last_second_against_the_bitrate)
{
    EXPECT_CALL(can, SendData(_, _, _)).Times(1000);
    for (int frame = 0; frame != 1000; ++frame)
        meter->SendData(hal::Can::Id::Create29BitId(1), Bytes(8), [](bool) {});
    for (int frame = 0; frame != 1000; ++frame)
        received(hal::Can::Id::Create29BitId(2), Bytes(0));
    ForwardTime(1s);

    ASSERT_TRUE(meter->Load().has_value());
    EXPECT_NEAR((1000.0f * (67.0f + 64.0f) + 1000.0f * 67.0f) / 1.0e6f, *meter->Load(), 1.0e-5f);
    EXPECT_EQ(1000u, delivered);

    ForwardTime(1s);
    EXPECT_NEAR(0.0f, *meter->Load(), 1.0e-6f);
}
