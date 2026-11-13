# e-robot-arm — Copilot / AI agent instructions

Concise, task-oriented guidance for AI coding agents. This repository is the
controller of a six-joint robot arm driving e-foc motor nodes over CAN, bootstrapped
from embedded-scaffold. See [`CLAUDE.md`](../CLAUDE.md) for the
full guide — this file is a short summary.

## 1. Architecture (short)

- `core/` — reusable libraries only (no entry points); `interfaces/` +
  `implementations/` (+ `test/`) + `test_doubles/`. See `core/joint_bus/`.
- `targets/` — application entry points (none yet) and platform
  implementations (`platform_implementations/{host,st,ti}`), selected by
  `E_ROBOT_ARM_TARGET_PLATFORM`.
- `tools/` — host-side developer tools that reuse `core/` libraries.
- `integration_tests/` — cucumber-cpp / Gherkin BDD tests.
- `infra/` — submodules: `embedded-infra-lib` (bounded containers, `emil_*` CMake
  helpers), `numerical-toolbox`, `robotics-toolbox`, `can-lite`, `e-foc-protocol`,
  `hal`.

## 2. Developer workflow

```bash
git clone --recursive <repo>
cmake --preset host
cmake --build --preset host-Debug
ctest --preset host
```

Embedded builds use the board presets in `CMakePresets.json`.

## 3. Constraints (must follow)

- **No heap** in `core/` runtime code, `targets/`, or ISR-reachable paths
  (`new/delete`, `malloc/free`, `make_unique`, dynamic STL containers are
  forbidden — use `infra::BoundedVector`/`infra::BoundedString`). Host tools/tests
  may use the heap.
- No pure virtual destructors (`virtual ~T() = 0`); use a protected non-virtual
  destructor on interfaces.
- Prefer fixed-size integer types; avoid recursion and virtual calls in hot paths.
- Keep non-trivial implementation in `.cpp`; favour `constexpr`/`inline`/`const`.

## 4. Patterns

- New component: design first in `documentation/design/`, then lay it out like
  `core/joint_bus/` (interfaces, implementations with tests, test doubles).
- Platform abstraction: application logic depends on interfaces; the platform
  `Board` (see `targets/platform_implementations/host/`) is injected via the
  constructor, never global state.
- Numerical algorithms: follow `infra/numerical-toolbox/` — implement float first,
  then Q15/Q31 variants, with typed GoogleTest suites.

## 5. Testing & CI

- Unit tests on host with GoogleTest; integration tests with cucumber-cpp.
- Always use `testing::StrictMock<>` — `NiceMock`/`NaggyMock` are **forbidden**.
- CI builds host + embedded, lints/formats, runs static analysis, and validates
  `documentation/`.

## 6. Performance

- See `documentation/performance-optimization/README.md`.
- Avoid virtual dispatch in hot paths; analyse with `arm-none-eabi-objdump -d -C`.
- No `#pragma GCC optimize` and no `optimize` attribute: GCC does not inline a callee whose
  options differ from its caller's. The consumer sets optimisation flags per translation unit.
- `OPTIMIZE_FOR_SPEED` (`numerical/math/CompilerOptimizations.hpp`) only on hot functions
  defined in a header (class body or template), never on one defined in a `.cpp`.

## 7. Operator tools (UI)

Before designing, reviewing or changing anything a person sees or operates under `tools/` (the simulator,
the console), follow `.claude/skills/ux-ui-expert/SKILL.md`: safety first, then state visibility, then the
common task; logic in view models tested without Qt; ui-cpp primitives only.
For the simulator's 3-D view and visual quality, also follow `.claude/skills/qt-3d-gui/SKILL.md`: Qt Quick 3D with
PBR materials, image-based lighting, soft shadows and SSAO, embedded in the ui-cpp Widgets shell.
