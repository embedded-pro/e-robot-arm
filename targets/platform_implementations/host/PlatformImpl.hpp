#ifndef TARGETS_PLATFORM_IMPLEMENTATIONS_HOST_PLATFORM_IMPL_HPP
#define TARGETS_PLATFORM_IMPLEMENTATIONS_HOST_PLATFORM_IMPL_HPP

#include "core/platform_abstraction/Platform.hpp"
#include "hal/generic/TimerServiceGeneric.hpp"
#include "infra/event/EventDispatcherWithWeakPtr.hpp"
#include "infra/stream/StringOutputStream.hpp"
#include "services/peripheral/SerialCommunicationLoopback.hpp"
#include "services/tracer/Tracer.hpp"

namespace application
{
    // Host implementation of platform::Platform. It uses stubs and host facilities
    // (a wall-clock timer service, a loopback serial channel) so the same
    // application can be built and exercised off-target.
    class PlatformImpl final
        : public platform::Platform
    {
    public:
        hal::GpioPin& StatusLed() override;
        hal::SerialCommunication& Communication() override;
        services::Tracer& Tracer() override;
        void Run() override;

    private:
        // A do-nothing GPIO so DebugLed has an output to drive on the host.
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

        infra::EventDispatcherWithWeakPtr::WithSize<50> eventDispatcher;
        hal::TimerServiceGeneric timerService;
        GpioStub led;
        services::SerialCommunicationLoopback loopback;
        infra::StringOutputStream::WithStorage<1024> stream;
        services::TracerToStream tracer{ stream };
    };
}

#endif
