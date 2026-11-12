#include "targets/platform_implementations/ti/PlatformImpl.hpp"

namespace application
{
    hal::GpioPin& PlatformImpl::StatusLed()
    {
        return launchPad.DebugLed();
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
