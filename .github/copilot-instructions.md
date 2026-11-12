# embedded-scaffold — Copilot / AI agent instructions

Concise, task-oriented guidance for AI coding agents. This repository is a
**template** for embedded C++ projects; the `example_*` components demonstrate
conventions and are meant to be replaced. See [`CLAUDE.md`](../CLAUDE.md) for the
full guide — this file is a short summary.

## 1. Architecture (short)

- `core/` — reusable libraries only (no entry points); `interfaces/` +
  `implementations/` (+ `test/`). See `core/example_component/`.
- `targets/` — application entry points (`example_app`) and platform
  implementations (`platform_implementations/{host,st,ti}`), selected by
  `SCAFFOLD_TARGET_PLATFORM`.
- `tools/` — host-side developer tools that reuse `core/` libraries.
- `integration_tests/` — cucumber-cpp / Gherkin BDD tests.
- `infra/` — submodules: `embedded-infra-lib` (bounded containers, `emil_*` CMake
  helpers), `numerical-toolbox`, `can-lite`, `hal`.

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

- New component: copy `core/example_component/`; interfaces vs implementations split.
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
- Avoid virtual dispatch in hot paths; use `#pragma GCC optimize("O3", "fast-math")`
  and `OPTIMIZE_FOR_SPEED` for critical files; analyse with `arm-none-eabi-objdump -d -C`.
