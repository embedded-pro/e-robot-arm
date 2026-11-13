# core

Reusable libraries only — **no application entry points** live here (those go in
[`targets/`](../targets)).

```
core/
├── joint_bus/                # arm::JointBus — the six e-foc joints on one CAN client
│   ├── interfaces/           #   JointBus.hpp, JointTransmission.hpp (INTERFACE library)
│   ├── implementations/      #   JointBusImpl + test/
│   └── test_doubles/         #   JointBusMock
├── safety/                   # arm::SafetySupervisor — limits, contract gate, trip and re-arm
├── motion/                   # arm::JointMotion — synchronised point-to-point moves
├── control_loop/             # arm::ControlLoop — the control period and its overruns
├── lifecycle/                # arm::ArmLifecycle — bring-up, enable, move, trip and re-arm
├── arm/                      # arm::Arm — builds all of the above from one CAN client
└── platform_abstraction/     # platform::Platform interface — the hardware seam
    ├── Platform.hpp          #   peripherals exposed to application logic
    └── test_doubles/         #   PlatformMock (gmock) for host unit tests
```

**The key pattern**: `platform_abstraction/` defines an abstract interface; the
concrete board wiring lives in [`targets/platform_implementations/`](../targets/platform_implementations).
Application logic depends only on the interface, so it compiles
once and runs on the host (tested with `PlatformMock`) and on every board.

## Conventions (copy these for new components)

- **Interfaces vs implementations**: keep pure interfaces in `interfaces/` as
  `INTERFACE` CMake libraries; put compiled code in `implementations/`.
- **Library naming**: `e_robot_arm.<area>.<layer>` (e.g.
  `e_robot_arm.joint_bus.implementations`).
- **Include paths** are rooted at the repository root, so headers are included as
  `#include "core/joint_bus/implementations/JointBusImpl.hpp"`.
- **No heap in runtime/embedded code**: use `infra::BoundedVector`/`infra::BoundedString`
  from `infra/embedded-infra-lib`. Host tools and tests may use the heap.
- **Tests** live in a `test/` subfolder and register with `emil_add_test`.

## Add a new component

1. Write its design in `documentation/design/` first (template: `documentation/templates/design.md`).
2. Lay it out like `joint_bus/`: `interfaces/`, `implementations/` with `test/`, and `test_doubles/`, each with
   its own `CMakeLists.txt` and library.
3. Add `add_subdirectory(<your_component>)` to this file.
