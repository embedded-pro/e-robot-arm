# platform_implementations/st

STM32 implementation of `platform::Platform`, shared by all supported ST board
presets: **STM32F407G-DISC1**, **STM32F429I-DISC1** and **NUCLEO-H563ZI**.

- **StatusLed** — LD3 (green) on PG13.
- **Communication** — `hal::UartStm` on USART1 (TX = PA9, RX = PA10), 115200 8N1.
  Boards without a virtual COM port on this UART require a USB-UART adapter
  connected to PA9/PA10/GND.
- **Tracer** — `services::TracerToStream` over the UART.
- **Run** — runs `main_::StmEventInfrastructure`. A first member (`ClockInit`) calls
  `HAL_Init()` + the board's default clock configuration function before any
  peripheral is constructed (8 MHz HSE on all supported boards).

The default clock header/init function is selected per board in
`CMakeLists.txt` (via `SCAFFOLD_ST_CLOCK_HEADER` / `SCAFFOLD_ST_CLOCK_INIT`,
keyed off `TARGET_MCU`). To support another STM32 board, add a `TARGET_MCU`
case there pointing at the matching `hal_st` clock header/function, and add a
preset in `CMakePresets.json`.
