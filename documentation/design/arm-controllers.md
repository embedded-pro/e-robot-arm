---
title: "Arm Controllers Design"
type: design
status: draft
version: 0.5.0
component: "arm-controllers"
date: 2026-11-16
---

| Field     | Value                  |
|-----------|------------------------|
| Title     | Arm Controllers Design |
| Type      | design                 |
| Status    | draft                  |
| Version   | 0.5.0                  |
| Component | arm-controllers        |
| Date      | 2026-11-16             |

> Which controller turns the motion's reference into the joints' setpoints, and how one is enabled, disabled and
> switched at run time without a setpoint step (phase 2, [#18](https://github.com/embedded-pro/e-robot-arm/issues/18)
> part 3). Behaviour only; diagrams are Mermaid.

---

## Responsibilities

**Is responsible for:**

- Holding the arm controllers the arm offers, each by an id and a name, each enabled or disabled, and at most one
  of them active.
- Enabling, disabling and selecting a controller, refusing a switch that would make the setpoints jump.
- Running the active controller once per control period while the arm streams setpoints: from the measured joints
  and the motion's reference to one setpoint set, sent through the safety supervisor.
- Reporting every change of the active controller.

**Is NOT responsible for:**

- Generating the reference: joint motion plans and samples it ([joint-motion.md](joint-motion.md)).
- Guarding the setpoints: the safety supervisor checks every set it is sent ([safety.md](safety.md)).
- Deciding when setpoints stream: the arm starts and stops the streaming with the lifecycle's `Enabled` and `Moving`
  states ([arm-lifecycle.md](arm-lifecycle.md)).
- Speed and torque feed-forward on a position setpoint, and the controller catalogue from robotics-toolbox: phase 5
  ([#21](https://github.com/embedded-pro/e-robot-arm/issues/21)) widens the setpoint set and adds controllers.
- Saving which controllers are enabled and active in the arm profile: a later item of the profile format
  ([arm-profile.md](arm-profile.md)).

---

## Component Details

### Part A — The controller

A controller takes the measured joints (positions and speeds from the latest telemetry) and the reference (the
joint positions the motion wants now) and returns a setpoint set. A controller runs the joints in one control mode
and its set matches it: six joint positions for a position-mode controller; six joint torques, with the reference
positions alongside for the trace, for a torque-mode controller. Phase 5 adds speed and torque feed-forward to the
position set.

A controller may keep state between periods. `Reset` gives it the measured joints to start from. A controller
declares whether it supports **bumpless transfer**: after a `Reset` its sets continue from the measured positions,
so taking over mid-move does not make the setpoints jump. A stateless controller that outputs the reference as it is
does not: the reference can be ahead of where the joints are.

Two controllers come first:

| Name           | Output                                                                                                | Bumpless |
|----------------|-------------------------------------------------------------------------------------------------------|----------|
| `pass-through` | The reference, unchanged: today's behaviour                                                           | no       |
| `smoothed`     | Each joint's reference through a first-order low-pass of time constant τ, reset to measured           | yes      |
| `gravity-pd`   | Joint torques: a PD law on the position error plus the gravity torque, τ = Kp·(q* − q) − Kd·q′ + G(q) | yes      |

`smoothed` filters each joint with an exponential moving average whose weight per period T is T / (τ + T)
(numerical-toolbox `ExponentialMovingAverage`). It trades tracking lag for gentler setpoints; τ is 50 ms in the
reference arm.

**The arm controls torque.** The e-foc drives run in torque mode and every arm controller runs on the host (the PC
now, a microcontroller later), built from robotics-toolbox's manipulator controllers and numerical-toolbox's control
blocks rather than written here. `gravity-pd` is the first: robotics-toolbox's `PdGravityCompensation` over the arm
model's dynamics ([arm-model.md](arm-model.md) Part F), with q* the reference, q and q̇ the measured joints and Kp
(N·m/rad) and Kd (N·m·s/rad) six-by-six gain matrices. It has no state, and when streaming starts the reference is
the measured position, so its first set is G(q) − Kd·q̇: it takes over without a step.

**Gains from a bandwidth.** The joints differ by more than ten times in the inertia they move, and the shoulder
and elbow move each other, so no diagonal gain set suits them all: gains sized from each joint's own inertia leave
the fast coupled mode of shoulder and elbow, whose inertia is much smaller, damped beyond what a 10 ms loop can
hold, and it oscillates. The profile names a bandwidth ω (rad/s) and a damping ratio ζ instead, and the gains are
the arm's inertia matrix scaled: Kp = ω²·J, Kd = 2·ζ·ω·J, with J the arm model's mass matrix at the home position
plus, on its diagonal, each motor's rotor inertia times its gear ratio squared (Teknic's when the profile names no
motor). Near home every mode of the arm then has its poles at ω with damping ζ; away from home the mass matrix
drifts from J, and the poles with it. Each joint lags a move by about 2·ζ/ω times its speed, and the damping per
control period, 2·ζ·ω·T, does not depend on any inertia. The bandwidth is bounded by the control period and the
telemetry's age: 30 rad/s at 10 ms leaves a wide margin.

`pass-through` and `smoothed` are what remains of the position path, where the drive closes the position loop; they
go when that path is removed, and computed torque (robotics-toolbox `ComputedTorqueControl`) and a per-joint PID
with gravity feed-forward (numerical-toolbox) join `gravity-pd` (phase 8d, [#31](https://github.com/embedded-pro/e-robot-arm/issues/31)).
Until then the profile's enable mode chooses the path.

### Part B — Registry and selection

The arm registers its controllers once, when it is built, up to a fixed capacity of four. Ids are their
registration order. A controller is registered disabled; the arm then enables all three and selects the default
for its control mode, the profile's enable mode: `pass-through` in position mode, `gravity-pd` in torque mode. A
controller of another mode cannot be selected: the joints run in one mode from enable to disable, so switching
between position and torque control means a profile with the other enable mode.

| Request      | Accepted when                                                                                  | Effect                                                              |
|--------------|------------------------------------------------------------------------------------------------|---------------------------------------------------------------------|
| enable *id*  | *id* is registered                                                                             | The controller may be selected                                      |
| disable *id* | *id* is registered                                                                             | It may not be selected; if it was active, no controller is active   |
| select *id*  | *id* is enabled, its control mode is the arm's, and no move runs or the controller is bumpless | It is `Reset` from the measured joints and becomes active; reported |

Selecting the active controller again is accepted and changes nothing. With no controller active the arm cannot
stream setpoints: a running move is aborted at its next period and the joints hold their last setpoint. That is how
disabling the active controller stops the arm; it is not an emergency stop, and the arm stays enabled.

### Part C — Streaming

While the arm streams, the active controller runs once per control period, in the control loop's setpoint phase,
after joint motion has advanced its reference:

1. The reference is the motion's sample while a move runs. Otherwise it is the last reference given: the target of
   the last move, where an aborted move stopped, or the measured positions when streaming started.
2. The measured joints are each joint's latest reading.
3. The active controller's set goes to the safety supervisor, as positions or torques by the controller's mode. A set the supervisor refuses, or a period with no
   active controller, stops the streaming and aborts the running move.

Streaming starts when the arm becomes `Enabled` (the active controller is `Reset` from the measured joints) and stops
when it leaves `Enabled` and `Moving`. So setpoints go out every period while the arm is enabled, not only during a
move: a controller with state, `smoothed` or phase 5's, keeps converging after the reference stops.

---

## Interfaces

### Provided

| Interface              | Purpose                                                     | Contract                                                                  |
|------------------------|-------------------------------------------------------------|---------------------------------------------------------------------------|
| Arm controller         | Reset from the measured joints; one setpoint set per period | No heap; a bumpless controller continues from the measured state          |
| Arm controllers        | List, find by name, enable, disable, select; the active one | Refuses as Part B says; changes nothing when it refuses                   |
| Arm controllers events | The active controller changed (to one, or to none)          | Exactly one per change                                                    |
| Streaming              | Engage, disengage, one step per period; the last set sent   | A step reports when the streaming stopped                                 |
| Reference              | The joint reference the active controller tracks            | Only while streaming; checked by the safety supervisor (safety.md Part F) |

### Required

| Interface         | Purpose                                        | Contract                                 |
|-------------------|------------------------------------------------|------------------------------------------|
| Safety supervisor | Latest readings; setpoint sets                 | Refuses sets while tripped or off limits |
| Joint motion      | Whether a move runs; the reference each period | Advanced before the controllers' step    |

---

## Data Model

| Entity          | Field         | Type / Unit     | Range        | Notes                                      |
|-----------------|---------------|-----------------|--------------|--------------------------------------------|
| Measured joints | positions     | radians         | finite       | Latest reading per joint                   |
| Measured joints | speeds        | radians/second  | finite       | Latest reading per joint                   |
| Setpoint set    | positions     | radians         | joint limits | Phase 5 adds speed and torque feed-forward |
| Setpoint set    | torques       | N·m, joint side | finite       | Torque-mode controllers only               |
| `gravity-pd`    | bandwidth     | rad/s           | above 0      | 30 by default; Kp = ω²·J                   |
| `gravity-pd`    | damping ratio | 1               | above 0      | 0.7 by default; Kd = 2·ζ·ω·J               |
| Registry entry  | id            | integer         | 0–3          | Registration order                         |
| Registry entry  | name          | text            | unique       | `pass-through`, `smoothed`                 |
| Registry entry  | enabled       | flag            |              | Registered disabled                        |
| `smoothed`      | time constant | seconds         | > 0          | 50 ms in the reference arm                 |

---

## State Machine

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Streaming : engage / reset active from measured
    Streaming --> Streaming : step / active controller's set sent
    Streaming --> Idle : disengage
    Streaming --> Idle : set refused or no active controller / move aborted
```

---

## Sequence Diagrams

```mermaid
sequenceDiagram
    participant L as Control loop
    participant M as Joint motion
    participant A as Arm controllers
    participant S as Safety supervisor
    L->>M: step (advance the reference)
    L->>A: step (reference)
    A->>S: latest readings
    A->>A: active controller: measured, reference to setpoints
    A->>S: setpoint set
    S-->>A: accepted
```

```mermaid
sequenceDiagram
    participant U as Console or simulator
    participant A as Arm controllers
    participant C as smoothed
    U->>A: select smoothed (move running)
    A->>C: reset (measured joints)
    A-->>U: active controller: smoothed
    Note over A,C: next period's set starts at the measured positions
```

---

## Constraints & Limitations

| Constraint | Value / Description                                                                      |
|------------|------------------------------------------------------------------------------------------|
| Capacity   | Four controllers                                                                         |
| Bumpless   | From the measured joints: the setpoints move by at most the tracking error at the switch |
| Traffic    | One setpoint set per control period while the arm is enabled                             |
| Memory     | No heap; every controller's state is fixed-size                                          |

---

## Open Questions

| # | Question                                                | Options                                            | Status                       |
|---|---------------------------------------------------------|----------------------------------------------------|------------------------------|
| 1 | Whether the profile stores the enabled and active ones  | Profile section; console and simulator choice only | open: with phase 5           |
| 2 | Whether a switch to a non-bumpless controller may blend | Refuse while moving; ramp from the old set         | decided: refuse while moving |
