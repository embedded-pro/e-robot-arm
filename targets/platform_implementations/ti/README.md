# platform_implementations/ti

TI (Tiva C) implementation of `platform::Platform`, targeting the **EK-TM4C123GXL**
LaunchPad.

- **StatusLed** — on-board green LED (PF3), via the `LaunchPad` BSP (which also
  configures the system clock).
- **Communication** — `hal::tiva::Uart` on UART0 (TX = PA1, RX = PA0), routed to the
  on-board ICDI USB virtual COM port — no extra wiring needed.
- **Tracer** — `services::TracerToStream` over the UART.
- **Run** — runs `instantiations::EventInfrastructure`.

Selected by the `EK-TM4C123GXL` preset. `hal::tiva::Uart` inherits a protected
constructor, so `PlatformImpl` uses a small `CliUart` wrapper to expose it.
