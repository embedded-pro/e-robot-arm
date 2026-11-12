#ifndef TARGETS_PLATFORM_IMPLEMENTATIONS_TI_PLATFORM_IMPL_HPP
#define TARGETS_PLATFORM_IMPLEMENTATIONS_TI_PLATFORM_IMPL_HPP

#include "core/platform_abstraction/Platform.hpp"
#include "hal_tiva/instantiations/EventInfrastructure.hpp"
#include "hal_tiva/instantiations/LaunchPadBsp.hpp"
#include "hal_tiva/tiva/Gpio.hpp"
#include "hal_tiva/tiva/Uart.hpp"
#include "infra/stream/OutputStream.hpp"
#include "services/tracer/StreamWriterOnSerialCommunication.hpp"
#include "services/tracer/Tracer.hpp"

namespace application
{
    // EK-TM4C123GXL (Tiva C LaunchPad) implementation of platform::Platform.
    //
    // - Status LED  : on-board green LED (PF3), via the LaunchPad BSP
    // - CLI UART    : UART0, TX = PA1, RX = PA0 (115200 8N1), routed to the on-board
    //                 ICDI USB virtual COM port — no extra wiring needed.
    class PlatformImpl final
        : public platform::Platform
    {
    public:
        hal::GpioPin& StatusLed() override;
        hal::SerialCommunication& Communication() override;
        services::Tracer& Tracer() override;
        void Run() override;

    private:
        // hal::tiva::Uart inherits its constructor from UartBase as protected; a
        // derived class may call it, so this thin wrapper exposes a public one.
        class CliUart
            : public hal::tiva::Uart
        {
        public:
            CliUart(uint8_t index, hal::tiva::GpioPin& tx, hal::tiva::GpioPin& rx, const Config& config)
                : hal::tiva::Uart(index, tx, rx, config)
            {}
        };

        instantiations::LaunchPad launchPad;   // configures the system clock and LEDs
        instantiations::EventInfrastructure eventInfrastructure;
        hal::tiva::GpioPin uartTx{ hal::tiva::Port::A, 1 };
        hal::tiva::GpioPin uartRx{ hal::tiva::Port::A, 0 };
        hal::tiva::UartBase::Config uartConfig;
        CliUart uart{ 0, uartTx, uartRx, uartConfig };
        services::StreamWriterOnSerialCommunication::WithStorage<256> streamWriter{ uart };
        infra::TextOutputStream::WithErrorPolicy stream{ streamWriter };
        services::TracerToStream tracer{ stream };
    };
}

#endif
