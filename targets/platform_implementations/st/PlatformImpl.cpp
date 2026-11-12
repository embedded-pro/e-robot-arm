#include "targets/platform_implementations/st/PlatformImpl.hpp"

// Referenced by the hal_st clock configuration. All currently supported ST board
// presets (see CMakePresets.json) have an 8 MHz HSE.
unsigned int hse_value = 8'000'000;

namespace application
{
    hal::GpioPin& PlatformImpl::StatusLed()
    {
        return statusLed;
    }

    hal::SerialCommunication& PlatformImpl::Communication()
    {
        return uart;
    }

    services::Tracer& PlatformImpl::Tracer()
    {
        return tracer;
    }

    void PlatformImpl::Run()
    {
        eventInfrastructure.Run();
    }
}
