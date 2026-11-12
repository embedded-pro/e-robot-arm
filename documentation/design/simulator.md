---
title: "Simulator Design"
type: design
status: draft
version: 0.1.0
component: "simulator"
date: 2026-11-12
---

| Field     | Value            |
|-----------|------------------|
| Title     | Simulator Design |
| Type      | design           |
| Status    | draft            |
| Version   | 0.1.0            |
| Component | simulator        |
| Date      | 2026-11-12       |

> The e-robot-arm Simulator's components, threads, targets and data flow (phase 8a,
> [#28](https://github.com/embedded-pro/e-robot-arm/issues/28)). The screen itself — layout, controls, flows,
> look — is in [tool-simulator.md](tool-simulator.md). Behaviour only; diagrams are Mermaid.

---

## Responsibilities

**Is responsible for:**

- Running the arm stack of `core/` (`arm::Arm`: joint bus, safety supervisor, joint motion, control loop) on a PC
  against a selectable target: simulated joints, a `virtual:` CAN bus, or the real arm on a CAN adapter.
- Keeping the arm on its own thread, so neither rendering nor the operator can delay a control period.
- Handing the operator interface a consistent snapshot of the arm, its events and its traces, and taking its
  commands.
- Simulating joints in-process, with a first-order lag and fault injection, for work without hardware.

**Is NOT responsible for:**

- Any arm rule: whether a move is allowed, what trips the arm and when it may be re-armed are decided by `core/`.
  The simulator shows the answer and its reason.
- Motor physics: the in-process joints follow their setpoint through a lag. Motor dynamics come with e-foc virtual
  motors (phase 7, [#23](https://github.com/embedded-pro/e-robot-arm/issues/23)) on the `virtual:` target.
- The arm's geometry and kinematics: phase 8a draws a generic six-joint arm; phase 3
  ([#19](https://github.com/embedded-pro/e-robot-arm/issues/19)) supplies the arm model.

---

## Component Details

### Part A — Libraries

| Library                     | Where                  | Qt  | Contents                                                                                      |
|-----------------------------|------------------------|-----|-----------------------------------------------------------------------------------------------|
| `e_robot_arm.simulated_arm` | `tools/simulated_arm/` | no  | `VirtualCanBus`, `SimulatedJoint` (lag, fault, silence), `SimulatedArm` (six joints, one bus) |
| `e_robot_arm.arm_host`      | `tools/arm_host/`      | no  | `ArmHost`: the arm and its target on a thread; commands in; snapshot, events, traces out      |
| `e_robot_arm.simulator.vm`  | `tools/simulator/vm/`  | no  | View models: controls, joint table, status, target, event log, arm scene                      |
| `e-robot-arm-simulator`     | `tools/simulator/`     | yes | Qt glue: main window on ui-cpp's `QtAppShell`, the Qt Quick 3D scene, docks, settings         |

The integration tests and the console use `simulated_arm` and `arm_host` too, so the simulator, the console and the
scenarios run the same host code. Everything below the Qt glue is tested without Qt.

### Part B — Threads

Two threads, never more for the arm:

1. **Arm thread.** EMIL's event loop (`EventDispatcherThreadAware`, `TimerServiceGeneric`), the CAN adapter or the
   in-process bus, the simulated joints and `arm::Arm`. Every `core/` call happens here.
2. **GUI thread.** Qt's event loop, the view models and the widgets. It never calls into `core/`.

They meet in `ArmHost` only:

- **Commands** go in through `ArmHost::Post`, which schedules a closure on the arm thread's dispatcher (EMIL's
  `Schedule` is safe from any thread). It returns at once.
- **State** comes out through a snapshot that the arm thread publishes after each telemetry phase and the GUI copies
  at display rate, under one mutex held only for the copy.
- **Events and traces** come out through two bounded queues, filled on the arm thread and drained by the GUI each
  frame. A full queue drops its oldest entry and counts the drop; the arm thread never waits.

### Part C — Targets

| Target           | CAN                                                  | Joints                                         | When                                    |
|------------------|------------------------------------------------------|------------------------------------------------|-----------------------------------------|
| Simulated joints | In-process `VirtualCanBus`                           | Six `SimulatedJoint`s on the arm thread        | Default; no hardware, no other process  |
| Virtual bus      | can-lite `virtual:<bus>`, hosting or joining the hub | Whatever joins: e-foc virtual motors (phase 7) | Virtual motors in other processes       |
| Real arm         | can-lite `socketcan:<if>` or `slcan:<port>`          | The real e-foc nodes                           | Hardware; distinct banner, confirmation |

A target switch builds everything anew; nothing carries over:

1. It is refused while a move is running, with that reason.
2. The control loop stops, every joint of the old target is sent Stop, and the host waits until the transmit queue
   drains or 100 ms pass.
3. The arm, its client and the adapter are destroyed; the new target, client and arm are built.
4. The new arm starts stopped, untripped and with its contracts unchecked.

### Part D — Simulated joints

A `SimulatedJoint` is an e-foc node on the in-process bus (a `FocMotorCategoryServer`), moved from
`integration_tests/support/`. It accepts every command the arm sends, answers telemetry, and:

- **Follows its setpoint through a first-order lag** with time constant τ. The position advances when telemetry is
  requested, from the time since the last update: `θ ← θ* + (θ − θ*)·e^(−Δt/τ)`, and its speed reads
  `(θ* − θ)/τ`. τ = 0 follows exactly, which is what the scenarios use. The simulator uses τ = 50 ms.
- **Takes injected failures:** `Fault(code)` reports a fault in its next telemetry, `Silence()` stops answering.
  The simulator offers both per joint, on the simulated target only.

### Part E — Arm host

`ArmHost` owns the arm thread and, on it, the target and the arm. Its operator-facing surface:

| Operation              | Effect                                                                                                                                                                     |
|------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `Post(command)`        | One of: check contracts, select position mode, start, stop, clear faults, emergency stop, re-arm, move, abort, inject fault, silence joint. Its result arrives as an event |
| `SwitchTarget(target)` | Part C; its result arrives as an event                                                                                                                                     |
| `Snapshot()`           | A copy of the latest snapshot                                                                                                                                              |
| `DrainEvents(sink)`    | Every event since the last drain, in order                                                                                                                                 |
| `DrainTrace(sink)`     | Every trace sample since the last drain, in order                                                                                                                          |

The emergency stop is posted ahead of any other queued command, so a burst of moves cannot delay it.

The console moves onto `ArmHost` too: its `Session` thread code in `tools/arm_console/main/Main.cpp` goes away, and
its synchronous `Execute` waits for the command's result event.

### Part F — Snapshot, events and traces

The snapshot is what the screen shows; events are what the log shows; traces are what the scopes plot.

- **Snapshot** — published after each telemetry phase, with a sequence number so the GUI can tell a stale copy.
- **Events** — contracts resolved, joint fault, joint silent, trip with its source, re-arm, move done, move aborted,
  command refused (with the command and the reason), target switched; each with the arm-thread time.
- **Trace samples** — one per control period: the time, the six commanded positions and the six measured positions.

The commanded positions are joint motion's last sampled setpoint. Joint motion gains a read-only accessor for it
(`JointMotion::Setpoint()`, empty when no move has run); nothing else in `core/` changes for the simulator.

---

## Interfaces

### Provided

| Interface      | Purpose                                 | Contract                                                       |
|----------------|-----------------------------------------|----------------------------------------------------------------|
| `ArmHost`      | The arm on its own thread               | Thread-safe; never blocks the caller beyond one snapshot copy  |
| View models    | Everything the screen shows and enables | GUI thread only; no Qt; tested with ui-cpp's recording backend |
| `SimulatedArm` | Six in-process e-foc joints on one bus  | Arm thread only                                                |

### Required

| Interface                          | Purpose                     | Contract                                           |
|------------------------------------|-----------------------------|----------------------------------------------------|
| `arm::Arm`                         | The arm stack               | Unchanged, plus `JointMotion::Setpoint()`          |
| can-lite `PlatformCanAdapter`, hub | Virtual and real targets    | `virtual:`, `socketcan:`, `slcan:`                 |
| ui-cpp (`infra/ui-cpp`)            | Shell, forms, scopes, theme | Tier 1 always built; Qt backend with the simulator |
| Qt 6 Widgets, Quick, Quick 3D      | Window, 3-D view            | Qt ≥ 6.4; 6.5+ features behind version guards      |

---

## Data Model

| Entity          | Field                 | Type / Unit                         | Range                 | Notes                                   |
|-----------------|-----------------------|-------------------------------------|-----------------------|-----------------------------------------|
| Snapshot        | sequence              | count                               | —                     | Increments per publish                  |
| Snapshot        | target                | simulated, virtual, real + name     | —                     | Name: bus or adapter                    |
| Snapshot        | readings              | 6 × (rad, rad/s, state, fault)      | —                     | Empty until the joint first answers     |
| Snapshot        | commanded             | 6 × rad, optional                   | Joint limits          | Joint motion's last setpoint            |
| Snapshot        | joint liveness        | 6 × answered, silent                | —                     | From the joint bus                      |
| Snapshot        | contracts             | unchecked, compatible, incompatible | —                     | Per joint                               |
| Snapshot        | tripped, moving, loop | bool                                | —                     |                                         |
| Event queue     | capacity              | events                              | 512                   | Oldest dropped when full; drops counted |
| Trace queue     | capacity              | samples                             | 1024 (10 s at 100 Hz) | Oldest dropped when full; drops counted |
| Simulated joint | τ                     | s                                   | ≥ 0                   | 0 follows exactly; simulator 0.05       |

---

## State Machine

```mermaid
stateDiagram-v2
    [*] --> Building
    Building --> Running : target and arm built
    Building --> Failed : adapter could not open
    Running --> Switching : switch target, no move running
    Switching --> Building : old target stopped and destroyed
    Failed --> Building : switch target
    Running --> [*] : shut down
```

---

## Sequence Diagrams

```mermaid
sequenceDiagram
    participant G as GUI thread
    participant H as ArmHost
    participant A as Arm thread
    participant J as Joints
    G->>H: Post(move)
    H->>A: schedule
    A->>A: arm Motion().MoveTo
    loop every control period
        A->>J: request telemetry
        J-->>A: readings
        A->>H: publish snapshot, trace sample
        A->>J: setpoints
    end
    Note over G: every display frame (≤ 60 Hz)
    G->>H: Snapshot(), DrainEvents(), DrainTrace()
    H-->>G: copies
```

---

## Block Diagram

```mermaid
flowchart LR
    subgraph GUI thread
        W[Qt widgets and Quick 3D scene] --> VM[View models]
    end
    subgraph Arm thread
        AR[arm::Arm] --> C[FocMotorCategoryClient]
        C --> T[Target: in-process bus, virtual: or real adapter]
        T --> SJ[Simulated joints]
    end
    VM -- Post, SwitchTarget --> H[ArmHost]
    H -- snapshot, events, traces --> VM
    H --- AR
```

---

## Constraints & Limitations

| Constraint | Value / Description                                                                              |
|------------|--------------------------------------------------------------------------------------------------|
| Timing     | Event-loop timers on a desktop OS: periods jitter with scheduling; real-time comes with phase 9  |
| Memory     | Host tools may allocate; `core/` stays heap-free; the queues are bounded                         |
| Bus        | Phase 1 adds bus load and overrun counts to the snapshot; until then the status shows "—"        |
| Lifecycle  | No arm lifecycle until phase 1; the controls follow the joints' states and the safety supervisor |

---

## Open Questions

| # | Question                                               | Options                                                  | Status            |
|---|--------------------------------------------------------|----------------------------------------------------------|-------------------|
| 1 | Whether the simulated joints should model speed limits | Lag only (8a), virtual motors for dynamics (phase 7)     | decided: lag only |
| 2 | A status-coloured read-only joint table in ui-cpp      | ui-cpp `ui::model` read-out table with a status per cell | open              |
