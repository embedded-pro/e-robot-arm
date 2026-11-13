# e-robot-arm — Claude Code Instructions

This file is a concise, task-oriented guide for Claude and AI agents to be
immediately productive in this repository. e-robot-arm was bootstrapped from
embedded-scaffold, whose example components phase 1 removed.
The planned architecture is in `documentation/architecture/system.md`; the phases still to build are in
`documentation/roadmap.md`, one GitHub issue per phase (tracking issue #16). A change that completes or reshapes a
phase updates the roadmap in the same pull request.

## 1. Big-Picture Architecture

**Purpose**: the controller of a six-joint robot arm whose joints are e-foc motor
nodes on one CAN bus. It runs on a PC for now; `core/` stays heap-free and
hardware-agnostic so it can move to a microcontroller (ARM Cortex-M) later. A Qt
simulator runs the same core against e-foc virtual motors or the real arm.

**Major components**:
- `core/` — reusable libraries only (no application entry points).
  - `core/joint_bus/` — `arm::JointBus`: the six e-foc joints on one
    `FocMotorCategoryClient` (node map, contract check, commands, setpoints,
    readings, fault and silence reports). Design: `documentation/design/joint-bus.md`.
  - `core/safety/` — `arm::SafetySupervisor`: a `JointBus` in front of the real
    one (joint limits, contract gate, emergency stop on a fault or silence,
    re-arm). Design: `documentation/design/safety.md`.
  - `core/motion/` — `arm::JointMotion`: synchronised trapezoidal point-to-point
    moves of the six joints, one setpoint set per control-loop step through the
    safety supervisor. Design: `documentation/design/joint-motion.md`.
  - `core/control_loop/` — `arm::ControlLoop`: the fixed control period; telemetry
    request to all joints, then the setpoint step half a period later.
    Design: `documentation/design/control-loop.md`.
  - `core/lifecycle/` — `arm::ArmLifecycle`: the start-up order (contracts,
    calibration, motor-controller configuration, enable, move) and trip/re-arm,
    on EMIL's table state machine; it
    follows the readings, so the single-step commands still work for debugging.
    Design: `documentation/design/arm-lifecycle.md`.
  - `core/joint_configurator/` — `arm::JointConfigurator`: applies and reads one
    joint's motor controller (algorithms and gains) and motor (model and drive
    limits, with presets) over CAN while the arm is Ready or Configuring, one
    request at a time. Design: `documentation/design/joint-configurator.md`.
  - `core/arm_model/` — `arm::ArmModel`: the arm's OPW geometry and link masses;
    tool pose, Jacobian, singularities, inverse kinematics within the joint limits,
    gravity torque, inverse dynamics and the mass matrix, on robotics-toolbox.
    Design: `documentation/design/arm-model.md`.
  - `core/arm_profile/` — reads a JSON arm profile into `ArmConfig` and writes it
    back, heap-free on EMIL's JSON parser. Design: `documentation/design/arm-profile.md`.
  - `core/arm/` — `arm::Arm`: builds the joint bus, safety supervisor, motion,
    lifecycle, joint configurator and control loop from one `FocMotorCategoryClient` and an
    `ArmConfig`; heap-free.
  - `core/platform_abstraction/` — the `platform::Platform` interface (the seam
    between portable logic and hardware) plus a `PlatformMock` for tests.
- `targets/` — application entry points and platform-specific implementations.
  - An application's `Main.cpp` includes `PLATFORM_IMPL_HEADER` (set per platform in
    `targets/CMakeLists.txt`) to construct the selected `PlatformImpl`, so one entry
    point serves host/st/ti. No application is defined yet.
  - `targets/platform_implementations/{host,st,ti}/PlatformImpl.*` — concrete
    `platform::Platform` implementations (host stubs; EK-TM4C123GXL; STM32F429I-DISC1).
  - The active platform is chosen by `E_ROBOT_ARM_TARGET_PLATFORM`.
- `tools/` — host-side developer tools that reuse `core/` libraries. Built only for the host.
  - `tools/arm_console/` — `e-robot-arm-console`: text commands to an
    `ArmHost` on simulated joints (`--bus simulated`), a `virtual:<bus>` (hosts or
    joins the virtual hub) or a real adapter. Design: `documentation/design/arm-console.md`.
  - `tools/simulated_arm/` — six simulated e-foc joints on an in-process
    `VirtualCanBus`, with a first-order lag and fault injection; used by the
    integration tests, the console and the simulator.
  - `tools/arm_host/` — `arm_host::ArmHost`: the arm on its own thread with its
    target (simulated, `virtual:` or a real adapter); commands in, snapshot,
    events and traces out. Design: `documentation/design/simulator.md`.
  - `tools/simulator/` — the e-robot-arm Simulator: `view_models/` (no Qt,
    tested with ui-cpp's recording backend) and `app/` (Qt Widgets on ui-cpp,
    the Qt Quick 3D scene as the `Simulator` QML module, built with the
    `host-qt` preset). Design: `documentation/design/simulator.md` and
    `tool-simulator.md`.
- `integration_tests/` — BDD tests (cucumber-cpp / Gherkin): `features/`, `steps/`,
  `support/`, `hooks/`, `main/`.
- `infra/numerical-toolbox/` — generic numerical algorithms (PID, filters,
  fixed-point helpers).
- `infra/embedded-infra-lib/` — infrastructure: bounded containers, build helpers
  (`emil_*` CMake functions), toolchain cmake pieces.
- `infra/robotics-toolbox/` — kinematics, dynamics, trajectories and manipulator
  control (`robotics/...` includes).
- `infra/e-foc-protocol/` — the e-foc motor CAN contract and `FocMotorCanClient`
  (`e-foc-protocol/...` includes).
- `infra/ui-cpp/` — UI primitives (forms, shell, scope, theme; Qt backend opt-in).
- `infra/can-lite/`, `infra/hal/` — CAN protocol library with host bus adapters, and
  hardware abstraction layer (st, ti).
- Every dependency is a submodule pinned to its `main`. e-foc-protocol and
  robotics-toolbox reuse this repository's embedded-infra-lib, can-lite and
  numerical-toolbox instead of fetching their own copies.

## 2. Critical Developer Workflows

```bash
# Clone (with submodules)
git clone --recursive <repo>

# Configure & build host (recommended first step)
cmake --preset host
cmake --build --preset host-Debug

# Run unit + integration tests
ctest --preset host

# Build for a board (compiles the platform libraries; no firmware application yet)
cmake --preset EK-TM4C123GXL
cmake --build --preset EK-TM4C123GXL-Debug
cmake --preset STM32F429I-DISC1
cmake --build --preset STM32F429I-DISC1-Debug

# Qt simulator (needs Qt Quick 3D); its window test needs a display (xvfb-run)
cmake --preset host-qt
cmake --build --preset host-qt-Debug
xvfb-run -a build/host-qt/tools/simulator/app/test/Debug/e_robot_arm.tool.simulator_window_test

# Coverage build
cmake --preset coverage
cmake --build --preset coverage
```

Configure presets (`CMakePresets.json`): `host`, `host-qt`, `coverage`, `EK-TM4C1294XL`,
`EK-TM4C123GXL`, `STM32F407G-DISC1`, `STM32F429I-DISC1`, `NUCLEO-H563ZI`.

## 3. Project-Specific Constraints (must follow)

### Memory — No Heap in Embedded/Real-Time Code

The no-heap rule applies to `core/` runtime code, `targets/`, and any ISR-reachable
path. Host tools, simulators, and test code may use normal heap patterns.

- No `new`, `delete`, `malloc`, `free`, `std::make_unique`, `std::make_shared`
- No `std::vector` → use `infra::BoundedVector<T>::WithMaxSize<N>`
- No `std::string` → use `infra::BoundedString::WithStorage<N>`
- No `std::deque`/`std::list`/`std::map`/`std::set` → use bounded alternatives from `embedded-infra-lib`
- No recursion in embedded/runtime paths (stack must be predictable)
- No `virtual ~Dtor() = 0` (pure virtual destructors) — adds flash/RAM overhead.
  Use a protected non-virtual destructor on interfaces (see
  `core/joint_bus/interfaces/JointBus.hpp`).

### Real-Time — Hot Paths

- No virtual dispatch or blocking calls in ISR-reachable hot paths.
- Never give a function optimisation options of its own: no `#pragma GCC optimize`,
  no `optimize` attribute. GCC does not inline a callee whose options differ from its
  caller's, so such a function calls every small helper it uses out of line. The
  consumer sets optimisation flags per translation unit.
- `OPTIMIZE_FOR_SPEED` (from `numerical/math/CompilerOptimizations.hpp`) forces
  inlining. Put it only on hot functions whose definition every caller sees (defined
  in the class body or as a template in a header), never on one defined in a `.cpp`.
- State explicit cycle budgets for timing-critical loops and verify with
  `arm-none-eabi-objdump -d -C`.

### Other Constraints

- Prefer fixed-size integer types (`uint8_t`, `int32_t`).
- Keep non-trivial logic in `.cpp` files; small `inline`/`constexpr` helpers in
  headers are allowed (and common in hot paths).
- Prefer `{}` initialization over `()`.
- `const` on all non-mutating methods; `constexpr` for constants and lookup tables.

## 4. Naming Conventions

- **Classes/Methods**: `PascalCase` (`ControlLoopImpl`, `Start()`, `OverrunCount()`)
- **Member variables**: `camelCase` (`halfPeriod`, `overruns`)
- **Namespaces**: lowercase (`arm`, `platform`, `integration`)
- **CMake libraries**: `e_robot_arm.<area>.<layer>`

## 5. Brace Style — Allman, 4-Space Indent

```cpp
namespace arm
{
    class ControlLoopImpl
        : public ControlLoop
    {
    public:
        void Start() override;
        uint32_t OverrunCount() const override;

    private:
        uint32_t overruns{ 0 };
    };
}
```

## 6. Patterns & Code Locations

- **New core component**: design it in `documentation/design/` first, then lay it out
  like `core/joint_bus/`: interfaces in `interfaces/`, implementations in
  `implementations/`, tests in `implementations/test/`, mocks in `test_doubles/`.
- **New application**: write the logic in `core/` against `platform::Platform`; add a
  thin `targets/<app>/Main.cpp` that includes
  `PLATFORM_IMPL_HEADER` and constructs it. Unit-test the logic on the host with
  `core/platform_abstraction/test_doubles/PlatformMock.hpp`.
- **New platform peripheral**: add a method to `platform::Platform`, implement it in
  each `targets/platform_implementations/<platform>/PlatformImpl`, and update the mock.
- **New board**: copy a `PlatformImpl` (e.g. `st/`), wire its peripherals via
  `infra/hal/<vendor>`, and add a preset in `CMakePresets.json`.
- **New tool**: lay it out like `tools/arm_console/`: logic in a library tested
  without I/O, a thin `main/`.
- **Include paths** are rooted at the repository root, e.g.
  `#include "core/joint_bus/implementations/JointBusImpl.hpp"`.

## 7. Testing

- Unit tests: `core/<component>/implementations/test/Test{Name}.cpp`, registered
  with `emil_add_test` / `emil_build_for`.
- Integration tests: cucumber-cpp under `integration_tests/`. Steps bind Gherkin
  lines via `GIVEN`/`WHEN`/`THEN`/`STEP`; share state through the cucumber `context`.
- Use `TEST_F` for fixtures; `TYPED_TEST` for multi-type tests.
- **`testing::StrictMock<>`** for ALL mock instances — `NiceMock`/`NaggyMock` are FORBIDDEN.
- Fixture class inside an anonymous `namespace {}`; test macros outside.
- Arrange-Act-Assert; `EXPECT_NEAR` with explicit tolerance for floating-point.

## 8. Documentation — Always Updated

- Documentation-first: update the relevant doc in `documentation/` before or
  alongside behavioural changes.
- `documentation/templates/` — starting points for new docs; CI validates
  architecture/design/theory docs and the requirements schema.
- All visuals must use Mermaid code blocks or ASCII art — no external image references.

## 9. Build System

- Presets are the primary interface (`CMakePresets.json`).
- Embedded toolchains: `infra/embedded-infra-lib/cmake/toolchain-*.cmake`.
- `compile_commands.json` is generated in build dirs for language servers/analysis.

## 10. Design Principles

- **SOLID** and **DRY**: reuse `infra/numerical-toolbox/` (PID, filters, transforms).
- **Constructor injection**: hardware dependencies via the platform `Board`/factory,
  never global state.
- **Error handling**: `std::optional<T>` for nullable returns; error codes/enums in
  embedded/runtime code (no exceptions); `assert()` for debug preconditions.

## Available Agents

Use the agents in `.claude/agents/` for structured development workflows:

- **orchestrator** — triage and route incoming development tasks
- **planner** — create detailed implementation plans before writing code
- **executor** — implement code changes following all project constraints
- **reviewer** — review code changes against project standards

Skills in `.claude/skills/`:

- **ux-ui-expert** (`.claude/skills/ux-ui-expert/SKILL.md`) — load before designing, reviewing or changing
  anything a person sees or operates under `tools/` (the simulator, the console): layouts, safety controls,
  configuration flows, the 3-D view, scopes. The simulator is built on ui-cpp primitives.
- **qt-3d-gui** (`.claude/skills/qt-3d-gui/SKILL.md`) — load with ux-ui-expert for the simulator's build and
  look: Qt Quick 3D scene (PBR materials, image-based lighting, soft shadows, SSAO, MSAA), embedding it in the
  Widgets application, live data at display rate, camera and picking, theme, typography, performance budget.
