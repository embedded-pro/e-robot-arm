# platform_implementations/host

Host implementation of `platform::Platform`. It satisfies the same interface as the
embedded boards using host facilities, so the application builds and is exercised
off-target:

- **StatusLed** — a no-op GPIO stub (so `DebugLed` has an output to drive).
- **Communication** — `services::SerialCommunicationLoopback` (TX loops back to RX).
- **Tracer** — `services::TracerToStream` over an in-memory string stream.
- **Run** — runs the host `EventDispatcher`; a `hal::TimerServiceGeneric` provides
  the system timer.

Built under the `host` preset. The portable logic it serves (`core/blinky_cli/`) is
also unit-tested against `PlatformMock` rather than this implementation.
