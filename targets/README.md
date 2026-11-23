# targets

Application **entry points** (`main`) and **platform-specific implementations**.
Libraries with no `main` belong in [`core/`](../core).

```
targets/
└── platform_implementations/
    ├── host/                     # PlatformImpl: stubs + loopback serial (host preset)
    ├── st/                       # PlatformImpl: STM32F429I-DISC1
    └── ti/                       # PlatformImpl: EK-TM4C123GXL
```

## How platform selection works

`E_ROBOT_ARM_TARGET_PLATFORM` (set per preset in `CMakePresets.json`) picks which
`platform_implementations/<platform>` subdirectory is compiled. Each provides a
`PlatformImpl : platform::Platform` (see `core/platform_abstraction/`) exposing the
peripherals the application needs.

`targets/CMakeLists.txt` builds an interface library `e_robot_arm.platform_impl`
that (a) defines `PLATFORM_IMPL_HEADER` pointing at the selected platform's
`PlatformImpl.hpp` and (b) links its library. A target's `Main.cpp` includes that
macro and constructs `application::PlatformImpl`, so the entry point stays
platform-agnostic: the same `Main.cpp` builds for host, ST and TI.

- **host** builds under `cmake --preset host`.
- **ti** builds the `EK-TM4C123GXL` platform; **st** builds `STM32F429I-DISC1`. No firmware
  application is defined yet; the arm controller is PC-hosted for now, and a release publishes no board firmware.

## Add a new application

1. Put the portable logic in `core/` against `platform::Platform`.
2. Add `targets/<your_app>/Main.cpp` that includes `PLATFORM_IMPL_HEADER` and constructs
   `application::PlatformImpl`; keep it platform-agnostic. For a board, link it with
   `halst_target_bringup` or `hal_ti_target_bringup` and the vendor's default linker scripts.
3. Add `add_subdirectory(<your_app>)` in `targets/CMakeLists.txt`.
4. To publish it with a release, add its ELF base name to `FIRMWARE_APPLICATIONS` in
   `.github/workflows/release-please.yml`. The release then requires exactly one `<your_app>.elf` per board and
   configuration, makes its `.hex`, and checks that both are in the board's archive before uploading it
   (`scripts/package-firmware.sh`, tested in CI by `scripts/test-package-firmware.sh`). Until then nothing is
   published for the boards, and an ELF that is built but not listed fails the release, so an application can't be
   left out of a release unnoticed.
