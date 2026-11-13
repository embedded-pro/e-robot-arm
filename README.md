# e-robot-arm

Real-time controller for a six-joint robot arm that drives e-foc motor nodes over CAN, with a Qt simulator that
runs the same code against virtual motors or the real hardware.

The repository was bootstrapped from
[`embedded-pro/embedded-scaffold`](https://github.com/embedded-pro/embedded-scaffold) and carries its structure,
build system, dev container, CI and documentation scaffolding.

> **Status**: the joint bus (`core/joint_bus/`) drives the six e-foc nodes over CAN, the safety supervisor
> (`core/safety/`) guards it, joint motion (`core/motion/`) moves all six joints together to a target, and the
> control loop (`core/control_loop/`) paces telemetry and setpoints, and the lifecycle (`core/lifecycle/`) walks the arm
> through bring-up, enable and re-arm; the joint configurator (`core/joint_configurator/`) applies and verifies each
> joint's motor controller. `core/arm/` puts them together and the
> `e-robot-arm-console` host tool and the simulator drive them over any can-lite adapter; the rest of the arm is
> planned in the [roadmap](documentation/roadmap.md).

## Overview

Each joint is an [e-foc](https://github.com/embedded-pro/e-foc) motor node that closes its own current, speed and
position loops. The arm controller runs on a PC for now: it plans and coordinates the six joints and talks to them
over CAN through [e-foc-protocol](https://github.com/embedded-pro/e-foc-protocol). Kinematics, dynamics and
trajectories come from [robotics-toolbox-cpp](https://github.com/embedded-pro/robotics-toolbox-cpp).

The controller is written once in `core/` against abstractions, so the same code runs against:

- a simulated arm, for fast development;
- six e-foc virtual motors on can-lite's `virtual:` bus, running the real motor firmware;
- the real arm, through a SocketCAN, slcan, PCAN or Kvaser adapter.

See the [system architecture](documentation/architecture/system.md) for the planned components.

## Getting Started

### Prerequisites

- Docker (the project is developed inside a Dev Container) **or** a local toolchain with CMake ≥ 3.24, Ninja, a
  C++20 compiler, Qt 6 and, for embedded targets, ARM GCC.

### Quick Start

```bash
git clone --recursive https://github.com/embedded-pro/e-robot-arm.git
cd e-robot-arm
cmake --preset host
cmake --build --preset host-Debug
ctest --preset host
```

### The simulator

The e-robot-arm Simulator runs the arm stack against six simulated joints, a `virtual:` bus or the real arm, with a
lit, shadowed 3-D arm (Qt Quick 3D), the joint table, the bring-up and move controls, a scope and an event log. It
needs Qt 6.4 or later with Qt Quick 3D (`qt6-quick3d-dev`, `qml6-module-qtquick3d` — `qml6-module-quick3d` on
Ubuntu 24.04 — `qt6-declarative-dev`, `qt6-shadertools-dev`):

```bash
cmake --preset host-qt
cmake --build --preset host-qt-Debug
build/host-qt/tools/simulator/app/Debug/e-robot-arm-simulator
```

Esc is the emergency stop from anywhere in the application.

### Driving the arm from a terminal

`e-robot-arm-console` opens a CAN adapter, builds the arm and reads one command per line (`contracts`,
`mode position`, `start`, `move q1 … q6`, `status`, `estop`, `clear`, `rearm`, `quit`, …):

```bash
build/host/tools/arm_console/Debug/e-robot-arm-console --bus simulated
```

`--bus` takes `simulated` (six simulated joints in the process), `virtual:<name>` (hosts or joins can-lite's
virtual CAN hub, where e-foc virtual motors connect), a SocketCAN interface such as `socketcan:can0`, or an slcan
adapter.

All presets are defined in `CMakePresets.json`. The scaffold's board presets (`EK-TM4C1294XL`, `EK-TM4C123GXL`,
`STM32F407G-DISC1`, `STM32F429I-DISC1`, `NUCLEO-H563ZI`) stay available for a later microcontroller target.

## Project Structure

```text
e-robot-arm/
├── core/                      # Reusable libraries only — no entry points
│   ├── joint_bus/             #   the six e-foc joints as one set of commands and readings
│   ├── safety/                #   joint limits and emergency stop in front of the joint bus
│   ├── motion/                #   synchronised point-to-point joint moves
│   ├── control_loop/          #   the fixed control period: telemetry, then setpoints
│   ├── lifecycle/             #   the start-up sequence: bring up, enable, move, trip and re-arm
│   ├── arm/                   #   the composition of the components above
│   └── platform_abstraction/  #   platform::Platform interface (+ mock)
├── targets/                   # Application entry points + platform implementations (host, st, ti)
├── tools/                     # Host-side tools: simulator, arm_console, arm_host, simulated_arm
├── integration_tests/         # BDD integration tests (cucumber-cpp / Gherkin)
├── infra/                     # Submodules
│   ├── embedded-infra-lib/    #   bounded containers, build helpers, toolchains
│   ├── numerical-toolbox/     #   numerical algorithms
│   ├── robotics-toolbox/      #   kinematics, dynamics, trajectories, manipulator control
│   ├── can-lite/              #   CAN protocol and host bus adapters
│   ├── e-foc-protocol/        #   e-foc motor CAN contract and client
│   └── hal/                   #   hardware abstraction layer (st, ti)
└── documentation/             # Architecture, design, theory, requirements (+ templates)
```

Every dependency is a submodule pinned to its `main`; e-foc-protocol and robotics-toolbox reuse this repository's
copies of embedded-infra-lib, can-lite and numerical-toolbox rather than fetching their own.

## Key Design Principles

- **No heap in runtime code**: bounded containers instead of STL containers. Host tools and tests may use the heap.
- **Interface-driven design + dependency injection**: hardware and buses are injected through constructors.
- **Documentation-first**: update the relevant `documentation/` document before or alongside behavioural changes.
  Diagrams use Mermaid or ASCII art only.
- **Reuse**: kinematics, dynamics and control from robotics-toolbox; numerics from numerical-toolbox; the joint
  protocol from e-foc-protocol.

## Documentation

| Document                                                                                 | Description                                            |
|------------------------------------------------------------------------------------------|--------------------------------------------------------|
| [Roadmap](documentation/roadmap.md)                                                      | Phases, issues and decisions still open                |
| [System Architecture](documentation/architecture/system.md)                              | Planned components, interfaces and decisions           |
| [Joint Bus Design](documentation/design/joint-bus.md)                                    | The six e-foc joints as one set of commands            |
| [Safety Supervisor Design](documentation/design/safety.md)                               | Limits and emergency stop in front of the bus          |
| [Joint Motion Design](documentation/design/joint-motion.md)                              | Synchronised point-to-point joint moves                |
| [Control Loop Design](documentation/design/control-loop.md)                              | Telemetry and setpoint phases of each period           |
| [Arm Lifecycle Design](documentation/design/arm-lifecycle.md)                            | The start-up sequence, trip and re-arm                 |
| [Synchronised Trapezoidal Moves](documentation/theory/synchronised-trapezoidal-moves.md) | The theory behind joint motion's moves                 |
| [Arm Composition and Console](documentation/design/arm-console.md)                       | Wiring the arm and driving it from a terminal          |
| [Simulator Design](documentation/design/simulator.md)                                    | The arm host, targets and threads behind the simulator |
| [Simulator Screen Design](documentation/design/tool-simulator.md)                        | Layout, controls, flows and look of the simulator      |
| [Documentation Templates](documentation/templates/)                                      | Starting points for new documents                      |
| [Performance Optimization Guide](documentation/performance-optimization/README.md)       | Embedded performance techniques and cycle budgets      |
| [AI Agent Instructions](CLAUDE.md)                                                       | Development guidelines, patterns and constraints       |

## License

See [LICENSE](LICENSE). Third-party components and submodules keep their own license terms.
