#include "tools/virtual_arm/plant_port/PlantPortCodec.hpp"
#include "tools/virtual_arm/plant_port/test/PlantPortBytes.hpp"
#include "gmock/gmock.h"

namespace
{
    using namespace virtual_arm::plant_port;
    using test::Bytes;

    class PlantPortCodecTest
        : public testing::Test
    {
    public:
        FrameAssembler assembler;
    };
}

TEST_F(PlantPortCodecTest, assembles_frames_split_across_reads)
{
    auto bytes = test::HelloFrame();
    const auto second = EncodeFrame(MessageType::refused, std::vector<uint8_t>{ 0x85, 0x00 });
    bytes.insert(bytes.end(), second.begin(), second.end());

    assembler.Append(std::span{ bytes }.first(2));
    EXPECT_FALSE(assembler.Next().has_value());
    assembler.Append(std::span{ bytes }.subspan(2));

    const auto hello = DecodeHello(*assembler.Next());
    ASSERT_TRUE(hello.has_value());
    EXPECT_EQ(2u, hello->version);
    EXPECT_EQ(20u, hello->node);
    EXPECT_EQ("arm", hello->bus);
    EXPECT_EQ("teknic", hello->plant);
    EXPECT_EQ(20000u, hello->controlRateHz);

    const auto refused = DecodeRefused(*assembler.Next());
    ASSERT_TRUE(refused.has_value());
    EXPECT_EQ(0x85u, refused->request);
    EXPECT_FALSE(assembler.Next().has_value());
}

TEST_F(PlantPortCodecTest, encodes_requests_in_the_wire_layout)
{
    EXPECT_EQ((std::vector<uint8_t>{ 0x85, 17, 0, 1, 0x00, 0x00, 0xc0, 0x3f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }), EncodeConstantExternalTorque(1.5f));
    EXPECT_EQ((std::vector<uint8_t>{ 0x90, 1, 0, 1 }), EncodeStream(true));
    EXPECT_EQ((std::vector<uint8_t>{ 0x81, 0, 0 }), EncodeFrame(MessageType::pause));
}

TEST_F(PlantPortCodecTest, a_payload_shorter_than_its_fields_does_not_decode)
{
    assembler.Append(Bytes{}.U16(2).U8(20).Frame(MessageType::hello));
    EXPECT_FALSE(DecodeHello(*assembler.Next()).has_value());

    assembler.Append(Bytes{}.U64(1).F32(1.0f).U32(0).U8(1).U8(0b101).U8(1).Frame(MessageType::status));
    const auto status = DecodeStatus(*assembler.Next());
    ASSERT_TRUE(status.has_value());
    EXPECT_TRUE(status->paused);
    EXPECT_EQ(0b101, status->leds);
}
