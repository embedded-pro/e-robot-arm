# embedded-scaffold

A template for embedded C++ projects with strict real-time and memory
constraints. It provides a ready-made structure, build system, dev container, CI,
and documentation scaffolding so a new firmware project can start from a working,
testable baseline instead of an empty repository.

> **Using this template**: click *Use this template* on GitHub (or clone), then
> replace the `example_*` placeholders in `core/`, `targets/`, `tools/`, and
> `integration_tests/` with your own components. Search the repo for
> `embedded_scaffold` / `embedded-scaffold` / `SCAFFOLD` to rename the project.

## Overview

The scaffold separates **portable logic** from **hardware** through one key seam:
an abstract `platform::Platform` interface in `core/platform_abstraction/`.
Application logic in `core/` depends only on that interface; each board provides a
concrete `PlatformImpl` in `targets/platform_implementations/`. The same
application is therefore compiled once and runs on the host (for testing) and on
each microcontroller.

The headline example is **`blinky_cli`**: blink a status LED and serve a tiny UART
command-line interface (`ping`, `id`). It is written once against the interface and
builds for the host, the **TI EK-TM4C123GXL**, and the **STM32F429I-DISC1**.

## Features

- **Platform abstraction**: application logic depends on the `platform::Platform`
  interface, not on an MCU — so it is built once and unit-tested on the host against
  a `PlatformMock`, then run on real hardware.
- **Real firmware examples**: `blinky_cli` builds to flashable `.elf`/`.hex` for the
  TI EK-TM4C123GXL and STM32F429I-DISC1 boards (LED blink + UART CLI).
- **No heap allocation** in runtime/embedded code — bounded containers from
  `infra/embedded-infra-lib` (`infra::BoundedVector`, `infra::BoundedString`).
- **Worked examples** in every top-level folder showing the conventions to follow.
- **Unit tests** (GoogleTest) and **BDD integration tests** (cucumber-cpp / Gherkin).
- **Dev container** with the full toolchain (CMake, Ninja, ccache, ARM GCC, Qt6).
- **CI** for build, linting/formatting, static analysis, documentation and
  requirements validation, and release-please versioning.

## Getting Started

### Prerequisites

- Docker (the project is developed inside a Dev Container) **or** a local toolchain
  with CMake ≥ 3.24, Ninja, a C++20 compiler, and (for embedded) ARM GCC.
- VS Code with the Dev Containers extension is the recommended workflow.

### Quick Start

1. Clone with submodules:
   ```bash
   git clone --recursive https://github.com/embedded-pro/embedded-scaffold.git
   cd embedded-scaffold
   ```

2. Configure & build for the host:
   ```bash
   cmake --preset host
   cmake --build --preset host-Debug
   ```

3. Run unit + integration tests:
   ```bash
   ctest --preset host
   ```

4. Run the example tool / app on the host:
   ```bash
   ./build/host/bin/Debug/embedded_scaffold.tool.example 2 3 4   # -> total = 9
   ./build/host/bin/Debug/embedded_scaffold.example_app          # -> accumulator total = 5
   ```

5. Build the `blinky_cli` firmware for a board (produces `.elf`/`.hex`):
   ```bash
   cmake --preset EK-TM4C123GXL          # TI Tiva C LaunchPad
   cmake --build --preset EK-TM4C123GXL-Debug

   cmake --preset STM32F429I-DISC1       # ST Discovery
   cmake --build --preset STM32F429I-DISC1-Debug
   ```
   On hardware the status LED blinks and a UART command-line interface (115200 8N1)
   accepts `ping` and `id`. On TI this is the on-board ICDI virtual COM port; on the
   F429 Discovery, wire a USB-UART adapter to PA9 (TX) / PA10 (RX) / GND.

All presets are defined in `CMakePresets.json` (`host`, `coverage`,
`EK-TM4C1294XL`, `EK-TM4C123GXL`, `STM32F407G-DISC1`, `STM32F429I-DISC1`,
`NUCLEO-H563ZI`).

## Project Structure

```
embedded-scaffold/
├── core/                      # Reusable libraries only — no entry points
│   ├── platform_abstraction/  #   platform::Platform interface (+ mock) — the seam
│   ├── blinky_cli/            #   portable app: LED blink + UART CLI (+ unit test)
│   └── example_component/     #   trivial interfaces/ + implementations/ (+ unit test)
├── targets/                   # Application entry points + platform implementations
│   ├── blinky_cli/            #   one Main.cpp reused across host/st/ti
│   ├── example_app/           #   a trivial host-only entry point
│   └── platform_implementations/
│       ├── host/              #   PlatformImpl: stubs + loopback serial (host build)
│       ├── st/                #   PlatformImpl: STM32F429I-DISC1 (LED PG13, USART1)
│       └── ti/                #   PlatformImpl: EK-TM4C123GXL (LaunchPad LED, UART0)
├── tools/                     # Host-side developer tools
│   └── example_tool/          #   a CLI reusing a core library
├── integration_tests/         # BDD integration tests (cucumber-cpp / Gherkin)
├── infra/                     # Infrastructure submodules
│   ├── embedded-infra-lib/    #   bounded containers, build helpers, toolchains
│   ├── numerical-toolbox/     #   PID, filters, fixed-point algorithms
│   ├── can-lite/              #   CAN protocol library
│   └── hal/                   #   hardware abstraction layer (st, ti)
├── documentation/             # Architecture/design/theory/requirements (+ templates)
├── scripts/                   # Build and utility scripts
└── build/                     # Build artifacts (generated, not committed)
```

Each top-level folder has its own `README.md` describing its conventions and how to
add new components.

## Key Design Principles

- **No heap in runtime code**: all memory statically allocated; bounded containers
  instead of STL containers. Host tools and tests may use the heap.
- **Interface-driven design + dependency injection**: hardware is injected via the
  `platform::Platform` interface through constructors, never global state.
- **Documentation-first**: update the relevant `documentation/` doc before or
  alongside behavioural changes. Diagrams use Mermaid or ASCII art only.
- **SOLID / DRY**: reuse `infra/numerical-toolbox/` algorithms; do not duplicate.

## Documentation

| Document | Description |
|---|---|
| [System Architecture](documentation/architecture/system.md) | Example architecture document |
| [Example Component Design](documentation/design/example-component.md) | Example design document |
| [Running Sum (Theory)](documentation/theory/example-topic.md) | Example theory document |
| [Example Requirements](documentation/requirements/example/example.yaml) | Example requirements (validated against `documentation/tools/requirement.schema.json`) |
| [Documentation Templates](documentation/templates/) | Starting points for new architecture/design/theory/requirements docs |
| [Performance Optimization Guide](documentation/performance-optimization/README.md) | Embedded performance techniques, assembly analysis, cycle budgets |
| [AI Agent Instructions](CLAUDE.md) | Development guidelines, patterns, and constraints |

## License

This project is licensed under the terms in the [LICENSE](LICENSE) file. Replace it
with the license appropriate for your project.

Third-party components and submodules keep their own license terms as documented in
their respective directories.
