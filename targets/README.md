# targets

Application **entry points** (`main`) and **platform-specific implementations**.
Libraries with no `main` belong in [`core/`](../core).

```
targets/
├── blinky_cli/                   # Headline app: one Main.cpp reused on every platform
│   └── Main.cpp                  #   includes PLATFORM_IMPL_HEADER, constructs BlinkyCli
├── example_app/                  # Trivial host-only entry point (Accumulator demo)
│   └── Main.cpp
└── platform_implementations/
    ├── host/                     # PlatformImpl: stubs + loopback serial (host preset)
    ├── st/                       # PlatformImpl: STM32F429I-DISC1
    └── ti/                       # PlatformImpl: EK-TM4C123GXL
```

## How platform selection works

`SCAFFOLD_TARGET_PLATFORM` (set per preset in `CMakePresets.json`) picks which
`platform_implementations/<platform>` subdirectory is compiled. Each provides a
`PlatformImpl : platform::Platform` (see `core/platform_abstraction/`) exposing the
peripherals the application needs.

`targets/CMakeLists.txt` builds an interface library `embedded_scaffold.platform_impl`
that (a) defines `PLATFORM_IMPL_HEADER` pointing at the selected platform's
`PlatformImpl.hpp` and (b) links its library. A target's `Main.cpp` includes that
macro and constructs `application::PlatformImpl`, so the entry point stays
platform-agnostic — the same `blinky_cli/Main.cpp` builds for host, ST and TI.

- **host** builds under `cmake --preset host`.
- **ti** builds the `EK-TM4C123GXL` firmware; **st** builds `STM32F429I-DISC1`.

## Add a new application

1. Put the portable logic in `core/` against `platform::Platform` (see `core/blinky_cli/`).
2. Copy `blinky_cli/` to `targets/<your_app>/`; keep `Main.cpp` platform-agnostic.
3. Add `add_subdirectory(<your_app>)` in `targets/CMakeLists.txt`.
