# core

Reusable libraries only — **no application entry points** live here (those go in
[`targets/`](../targets)).

```
core/
├── platform_abstraction/     # platform::Platform interface — the hardware seam
│   ├── Platform.hpp          #   peripherals exposed to application logic
│   └── test_doubles/         #   PlatformMock (gmock) for host unit tests
├── blinky_cli/               # Portable app: LED blink + UART command-line interface
│   ├── BlinkyCli.{hpp,cpp}   #   depends ONLY on platform::Platform
│   └── test/                 #   unit-tested on the host against PlatformMock
└── example_component/        # Trivial component showing the interfaces/impl split
    ├── interfaces/           #   Abstract INTERFACE libraries (Accumulator.hpp)
    └── implementations/      #   Compiled libraries (AccumulatorImpl) + test/
```

**The key pattern**: `platform_abstraction/` defines an abstract interface; the
concrete board wiring lives in [`targets/platform_implementations/`](../targets/platform_implementations).
Application logic such as `blinky_cli` depends only on the interface, so it compiles
once and runs on the host (tested with `PlatformMock`) and on every board.

## Conventions (copy these for new components)

- **Interfaces vs implementations**: keep pure interfaces in `interfaces/` as
  `INTERFACE` CMake libraries; put compiled code in `implementations/`.
- **Library naming**: `embedded_scaffold.<area>.<layer>` (e.g.
  `embedded_scaffold.example.implementations`).
- **Include paths** are rooted at the repository root, so headers are included as
  `#include "core/example_component/implementations/AccumulatorImpl.hpp"`.
- **No heap in runtime/embedded code**: use `infra::BoundedVector`/`infra::BoundedString`
  from `infra/embedded-infra-lib`. Host tools and tests may use the heap.
- **Tests** live in a `test/` subfolder and register with `emil_add_test`.

## Add a new component

1. Copy `example_component/` to `core/<your_component>/`.
2. Rename the libraries and update `target_sources`.
3. Add `add_subdirectory(<your_component>)` to this file.
