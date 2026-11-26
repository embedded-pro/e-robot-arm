# e-robot-arm — Agent Rules

<!-- shared:begin 0.7.0 sha256:000f44de0ff3a5bd -->
Canonical rules for Claude, Copilot and sub-agents. Sections marked *(shared)* come from embedded-pro/embedded-agent-kit:
edit them there, not here. *(repo)* sections belong to this repository. *Exceptions* overrides a shared rule.

## Behaviour *(shared)*

- **Be as terse as possible.** No preamble, summary or narration. Report file paths and pass/fail.
- Everything you write — code, docs, agent files, PR text — uses the fewest words that stay unambiguous.
- Prose (docs, comments, agent files, commit and PR text) follows ASD-STE100 Simplified Technical English:
  one word, one meaning; active voice; one instruction per sentence; sentences ≤ 20 words (procedures) or ≤ 25 (descriptions).
- Ambiguous request: ask. Clear request: act.
- Do only what was asked. No unrelated refactors, no extra features.
- Batch reads, don't re-read, prefer targeted edits.
- Never change a shared section. Add a deviation to *Exceptions* only when the user approves it.

## Workflow *(shared)*

1. Multi-file or architectural change: update the design document first. Get approval before you write code.
2. Read *Domain* and *Build*. Read each `.github/instructions/*.instructions.md` whose `applyTo` matches a file you change.
3. Find the nearest existing pattern. Follow it.
4. Per behaviour: write a failing test, write the minimum code that passes it, refactor with all tests green.
5. Add each new file to its `CMakeLists.txt`.
6. Update the documentation for each behavioural change.
7. Run the *Build* commands until build and tests pass. Never disable, skip or weaken a test to get green.
8. Non-trivial change: run the `reviewer` agent, or ask the user to run it. Fix each CRITICAL and WARNING finding.
9. Report changed files, build pass/fail, test pass/fail.

## Memory *(shared)*

Scope: production code. Tests and host tools may use the heap.

- Forbidden: `new`/`delete`/`malloc`/`free`, `make_unique`/`make_shared`,
  `std::vector`/`string`/`deque`/`list`/`map`/`set`, recursion.
- Use: `infra::BoundedVector<T>::WithMaxSize<N>`, `infra::BoundedString::WithStorage<N>`,
  `infra::BoundedDeque`, `infra::BoundedList`/`IntrusiveList`, `std::array`, `std::optional`, `std::variant`.

## Interfaces & errors *(shared)*

- No virtual destructors: each adds a deleting destructor and thunks to flash.
  Interface = pure virtual methods + `protected: ~I() = default;` (non-virtual).
- Public `virtual ~Base() = default;` only on a base destroyed through a base pointer:
  `delete`/`std::unique_ptr<Base>`, `infra::PolymorphicVariant<Base, ...>`, `infra::OptionalForPolymorphicObjects<Base, ...>`.
  Put it on that base, not on every interface.
- Not needed for `std::variant`, `std::optional<Concrete>`, `infra::SharedPtr`: they destroy the concrete type.
- No exceptions. `std::optional`, `bool` or status enum; `really_assert` for preconditions. Never swallow an error.
- Constructor injection; no global state.

## Optimisation *(shared)*

- No `#pragma GCC optimize`, no `optimize` attribute: GCC won't inline across differing options.
  The consumer sets flags per translation unit.
- `OPTIMIZE_FOR_SPEED` only on hot functions defined in a header.
- Hot paths: no heap, no virtual dispatch, no blocking.

## Style *(shared)*

- `.clang-format` is authoritative. Allman, 4 spaces, `{}` init.
- PascalCase types/methods; camelCase members and enum values; lowercase namespaces.
  No `I` prefix; implementations end in `Impl`.
- `#pragma once` only.
- Fixed-width integers. `const`/`constexpr`-correct. Functions ≤ 30 lines (hard limit 50).
- Comments: non-obvious *why* only, one line. No `TODO`/`FIXME`, commented-out code, or restating the code.
  Fix or delete a comment the change makes stale.

## Testing *(shared)*

- `TEST_F` for fixtures, `TYPED_TEST` for types, plain `TEST()` for stateless checks.
- `StrictMock` only. `EXPECT_NEAR` with explicit tolerance. Fixture in an anonymous namespace, macros outside.
- Heap allowed in tests.

## Documentation *(shared)*

- Update docs before or with any behavioural change.
- One fact, one place: link, never restate.
- Visuals: Mermaid or ASCII.
- Put each document in the folder for its kind: `theory/` (physics, mathematics, algorithms),
  `architecture/` (structure and deployment), `design/` (one component each),
  `verification_and_validation/` (test plans, test design, requirements).
- A test strategy or test plan is never a design document.
- Every document starts with front matter: `title`, `type`, `status`, `version`, `date`.
- A design document describes the component: responsibilities, behaviour, interfaces, states, limits.
  It names no MCU, board, vendor peripheral, register, clock frequency, class, function or file.
  Say "the target". Say "on the reference board" for a measured figure.
- Only an architecture document names the deployment target.
- A design document opens with `## Responsibilities`: one list of what the component does.
  No "not responsible for" list. Put boundaries in *Interfaces* (Required).
- No "Open Questions" section. Open an issue and link it. Put a known limitation in *Constraints & Limitations*.
- After a documentation change, build the rendered output if the repository has a build for it.
  Check that each diagram renders and that no table column runs into the next.
- Skill `doc-writer`, when installed: load it before you create or change a document.

## Desktop tools *(shared)*

Scope: host tools that people install and run, GUI or console.

- Open the log file first in `main`, before the application object.
  A GUI-subsystem build (`WIN32_EXECUTABLE`) has no stderr: the log file is its only output.
- Write the log to a per-user writable folder (`AppLocalDataLocation`) and rotate it.
  Capture the toolkit's messages (`qInstallMessageHandler`) from debug level.
- Install a crash handler after the log. It writes a `FATAL` line on a crash signal, `std::terminate`
  or an unhandled SEH exception, and a minidump on Windows. Qt tools use ui-cpp's `IssueLog` and `CrashReporter`.
- Log each start-up step: version, OS, toolkit version, arguments, restored settings, first frame.
- Provide `--safe-start`: it skips the saved state, so a bad saved state cannot block start-up.
- Give the log folder a menu item (Help ▸ Open Log Folder).
- Install test: start the tool, close it normally, then start it again with saved state.
  Print the log. On a failure, upload the log folder and the dumps.
<!-- shared:end -->

## Domain *(repo)*

- Controller of a six-joint robot arm. Each joint is an e-foc motor node on one CAN bus. It runs on a PC now;
  `core/` stays heap-free and hardware-agnostic so it can move to a Cortex-M later. A Qt simulator runs the same
  core against e-foc virtual motors or the real arm.
- Architecture: `documentation/architecture/system.md`. Phases still to build: `documentation/roadmap.md`, one
  GitHub issue per phase (tracking issue #16). A change that completes or reshapes a phase updates the roadmap in
  the same pull request.
- `core/` — libraries only, no entry points. Namespace `arm`. Design documents in `documentation/design/`:
  - `joint_bus/` — `arm::JointBus`: the six joints on one `FocMotorCategoryClient` (node map, contract check,
    commands, setpoints, readings, fault and silence reports). `joint-bus.md`.
  - `safety/` — `arm::SafetySupervisor`: a `JointBus` in front of the real one (joint limits, contract gate,
    emergency stop on a fault or silence, re-arm). `safety.md`.
  - `motion/` — `arm::JointMotion`: synchronised trapezoidal point-to-point moves, one setpoint set per
    control-loop step through the safety supervisor. `joint-motion.md`.
  - `control_loop/` — `arm::ControlLoop`: the fixed control period; telemetry request to all joints, then the
    setpoint step half a period later. `control-loop.md`.
  - `lifecycle/` — `arm::ArmLifecycle` on EMIL's table state machine: start-up order (contracts, calibration,
    motor-controller configuration, enable, move) and trip/re-arm. It follows the readings, so single-step
    commands still work for debugging. `arm-lifecycle.md`.
  - `joint_configurator/` — `arm::JointConfigurator`: applies and reads one joint's motor controller (algorithms,
    gains) and motor (model, drive limits, presets) over CAN while the arm is Ready or Configuring, one request at
    a time. `joint-configurator.md`.
  - `arm_model/` — `arm::ArmModel` on robotics-toolbox: OPW geometry and link masses; tool pose, Jacobian,
    singularities, inverse kinematics within the joint limits, gravity torque, inverse dynamics, mass matrix.
    `arm::DrivenArm` puts each drive's reflected rotor inertia on its link as armature; the arm controllers and
    the collision observer share one. `arm-model.md`.
  - `workspace/` — `arm::WorkspaceCheck`: a capsule per link, checked against the workspace box, forbidden zones
    and itself (`ConvexCollision`, `SelfCollision`). Joint motion refuses moves that fail it; the safety
    supervisor stops on it. `safety.md` Part H.
  - `collision/` — `arm::CollisionObserver`: robotics-toolbox momentum observer over the arm model and the
    reflected rotor inertia; estimates each joint's external torque for the safety collision check.
    `collision.md`.
  - `arm_profile/` — reads a JSON arm profile into `ArmConfig` and writes it back, heap-free on EMIL's JSON
    parser. `arm-profile.md`.
  - `arm/` — `arm::Arm`: builds joint bus, safety supervisor, motion, lifecycle, joint configurator and control
    loop from one `FocMotorCategoryClient` and an `ArmConfig`; heap-free.
  - `platform_abstraction/` — `platform::Platform`, the seam to the hardware, and `PlatformMock` for tests.
- `targets/` — entry points and platform implementations. An application's `Main.cpp` includes
  `PLATFORM_IMPL_HEADER` (set per platform in `targets/CMakeLists.txt`); one entry point serves host/st/ti. No
  application exists yet. `targets/platform_implementations/{host,st,ti}/PlatformImpl.*` are the
  `platform::Platform` implementations (host stubs; EK-TM4C123GXL; STM32F429I-DISC1).
  `E_ROBOT_ARM_TARGET_PLATFORM` selects one.
- `tools/` — host-only tools on `core/` libraries:
  - `arm_console/` — `e-robot-arm-console`: text commands to an `ArmHost` on simulated joints
    (`--bus simulated`), a `virtual:<bus>` (hosts or joins the virtual hub) or a real adapter. `arm-console.md`.
  - `simulated_arm/` — six simulated e-foc joints on an in-process `VirtualCanBus`, with a first-order lag and
    fault injection. Used by the integration tests, the console and the simulator.
  - `arm_host/` — `arm_host::ArmHost`: the arm on its own thread with its target (simulated, `virtual:` or a real
    adapter); commands in, snapshot, events and traces out. `simulator.md`.
  - `tuning/` — `tuning::TuningRun`: a step or out-and-back test on one joint through the arm host, its metrics
    (numerical-toolbox step-response metrics, RMS tracking error) and CSV. `tuning.md`.
  - `simulator/` — the e-robot-arm Simulator: `view_models/` (no Qt, tested with ui-cpp's recording backend) and
    `app/` (Qt Widgets on ui-cpp; the Qt Quick 3D scene is the `Simulator` QML module; `host-qt` preset).
    `simulator.md`, `tool-simulator.md`.
  - `installer/` — the tools `.deb` (Simulator, console, virtual arm launcher, e-foc's virtual motor), built and
    install-tested by `build-tools.yml` on every pull request and attached to releases. `tool-installers.md`.
- `integration_tests/` — cucumber-cpp: `features/`, `steps/`, `support/`, `hooks/`, `main/`. Steps bind Gherkin
  lines with `GIVEN`/`WHEN`/`THEN`/`STEP` and share state through the cucumber `context`.
- `infra/` — submodules, each pinned to its `main`: `embedded-infra-lib` (bounded containers, `emil_*` CMake,
  toolchains), `numerical-toolbox` (PID, filters, math; reuse it), `robotics-toolbox` (`robotics/...`),
  `e-foc-protocol` (motor CAN contract, `FocMotorCanClient`), `ui-cpp` (UI primitives, Qt backend opt-in),
  `can-lite`, `hal` (st, ti). e-foc-protocol and robotics-toolbox reuse this repository's embedded-infra-lib,
  can-lite and numerical-toolbox.
- *Memory* covers `core/` runtime code, `targets/` and every ISR-reachable path. Host tools, simulators and tests
  may use the heap.
- Hot paths: no blocking calls in ISR-reachable code. State the cycle budget of each timing-critical loop; check
  it with `arm-none-eabi-objdump -d -C`.
- `OPTIMIZE_FOR_SPEED` comes from `numerical/math/CompilerOptimizations.hpp`. Never on a function defined in a
  `.cpp`.
- Non-trivial logic in `.cpp`; small `inline`/`constexpr` helpers in headers are allowed.
- Hardware comes in through the platform `Board`/factory. `std::optional<T>` for nullable returns; status enums
  in runtime code.
- New core component: design it in `documentation/design/` first. Lay it out like `core/joint_bus/`:
  `interfaces/`, `implementations/`, `implementations/test/`, `test_doubles/`.
- New application: logic in `core/` against `platform::Platform`, unit-tested with `PlatformMock`; a thin
  `targets/<app>/Main.cpp` includes `PLATFORM_IMPL_HEADER` and constructs it.
- New peripheral: add a method to `platform::Platform`, implement it in each `PlatformImpl`, update the mock.
- New board: copy a `PlatformImpl` (e.g. `st/`), wire it through `infra/hal/<vendor>`, add a preset.
- New tool: lay it out like `tools/arm_console/`: logic in a library tested without I/O, a thin `main/`.
- Includes are rooted at the repository root (`#include "core/joint_bus/implementations/JointBusImpl.hpp"`).
  CMake libraries: `e_robot_arm.<area>.<layer>`.
- Unit tests: `core/<component>/implementations/test/Test{Name}.cpp`, registered with `emil_add_test` /
  `emil_build_for`. Host stubs: `targets/platform_implementations/host/`.
- Documents: `documentation/{architecture,design,theory,requirements}/`; start one from
  `documentation/templates/`. CI validates architecture, design and theory documents and the requirements schema.
- Operator tools: before you design, review or change anything a person sees or operates under `tools/`, load
  the `ux-ui-expert` skill (`.claude/skills/ux-ui-expert/SKILL.md`). For the simulator's 3-D view, build and look,
  also load `qt-3d-gui` (`.claude/skills/qt-3d-gui/SKILL.md`).

## Build *(repo)*

```bash
git submodule update --init --recursive
cmake --preset host && cmake --build --preset host-Debug
ctest --preset host
cmake --preset EK-TM4C123GXL && cmake --build --preset EK-TM4C123GXL-Debug
cmake --preset STM32F429I-DISC1 && cmake --build --preset STM32F429I-DISC1-Debug
cmake --preset host-qt && cmake --build --preset host-qt-Debug
xvfb-run -a build/host-qt/tools/simulator/app/test/Debug/e_robot_arm.tool.simulator_window_test
cmake --preset coverage && cmake --build --preset coverage
```

- Board presets build the platform libraries only; no firmware application exists yet.
- `host-qt` needs Qt Quick 3D. The simulator window test needs a display: run it under `xvfb-run`.
- Presets: `host`, `host-qt`, `host-single-Debug`, `windows`, `coverage`, `EK-TM4C1294XL`, `EK-TM4C123GXL`,
  `STM32F407G-DISC1`, `STM32F429I-DISC1`, `NUCLEO-H563ZI`. `compile_commands.json` is in each build directory.

## Exceptions *(repo)*

None.
