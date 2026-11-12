#include "core/blinky_cli/BlinkyCli.hpp"
#include "core/platform_abstraction/test_doubles/PlatformMock.hpp"
#include "hal/interfaces/test_doubles/SerialCommunicationMock.hpp"
#include "infra/stream/StringOutputStream.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "services/tracer/Tracer.hpp"
#include "gmock/gmock.h"
#include <string>

namespace
{
    // Minimal GpioPin so DebugLed has a real output to drive in the host test.
    class GpioStub final
        : public hal::GpioPin
    {
    public:
        bool Get() const override
        {
            return false;
        }
        void Set(bool) override
        {}
        bool GetOutputLatch() const override
        {
            return false;
        }
        void SetAsInput() override
        {}
        bool IsInput() const override
        {
            return false;
        }
        void Config(hal::PinConfigType) override
        {}
        void Config(hal::PinConfigType, bool) override
        {}
        void ResetConfig() override
        {}
        void EnableInterrupt(const infra::Function<void()>&, hal::InterruptTrigger, hal::InterruptType) override
        {}
        void DisableInterrupt() override
        {}
    };

    class BlinkyCliTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        BlinkyCliTest()
        {
            EXPECT_CALL(platform, StatusLed()).WillRepeatedly(testing::ReturnRef(led));
            EXPECT_CALL(platform, Communication()).WillRepeatedly(testing::ReturnRef(communication));
            EXPECT_CALL(platform, Tracer()).WillRepeatedly(testing::ReturnRef(tracer));
        }

        std::string Output() const
        {
            return std::string{ text.begin(), text.end() };
        }

        GpioStub led;
        hal::SerialCommunicationMock communication;
        infra::BoundedString::WithStorage<512> text;
        infra::StringOutputStream stream{ text };
        services::TracerToStream tracer{ stream };
        testing::StrictMock<platform::PlatformMock> platform;
    };
}

TEST_F(BlinkyCliTest, greets_and_shows_a_prompt_on_construction)
{
    application::BlinkyCli blinkyCli{ platform };

    EXPECT_THAT(Output(), testing::HasSubstr("blinky-cli ready"));
    EXPECT_THAT(Output(), testing::HasSubstr("> "));
}
