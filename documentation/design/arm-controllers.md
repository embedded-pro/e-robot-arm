---
title: "Arm Controllers Design"
type: design
status: draft
version: 0.14.0
component: "arm-controllers"
date: 2026-11-20
---

| Field     | Value                  |
|-----------|------------------------|
| Title     | Arm Controllers Design |
| Type      | design                 |
| Status    | draft                  |
| Version   | 0.14.0                 |
| Component | arm-controllers        |
| Date      | 2026-11-20             |

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
- Deciding when setpoints stream: the arm starts and stops the streaming with the lifecycle's `Enabling`, `Enabled`,
  `Moving` and `Holding` states ([arm-lifecycle.md](arm-lifecycle.md)).
- The rest of the controller catalogue from robotics-toolbox and numerical-toolbox (computed torque, a per-joint
  PID with gravity feed-forward): phase 8d step 6 ([#31](https://github.com/embedded-pro/e-robot-arm/issues/31)).
- Saving which controllers are enabled and active in the arm profile: a later item of the profile format
  ([arm-profile.md](arm-profile.md)).

---

## Component Details

### Part A — The controller

A controller takes the measured joints (positions and speeds from the latest telemetry; a reading from an earlier
period, answered too late for this step's setpoints, is advanced by its speed for each period it missed, at most two,
so a late answer does not add a period of delay to the loop) and the reference (the joint
positions, speeds and accelerations the motion wants now, [joint-motion.md](joint-motion.md) Part B) and returns a
setpoint set: six joint torques, with the reference positions alongside for the trace and for the safety
supervisor's tracking check.

A controller may keep state between periods. `Reset` gives it the measured joints to start from and the reference
held at that moment: the last one the old controller followed, or the measured pose at rest when streaming starts. A controller
declares whether it supports **bumpless transfer**: after a `Reset` its torques continue from the measured state,
so taking over mid-move does not make them jump.

| Name              | Output                                                                                                | Bumpless |
|-------------------|-------------------------------------------------------------------------------------------------------|----------|
| `gravity-pd`      | Joint torques: a PD law on the position error plus the gravity torque, τ = Kp·(q* − q) − Kd·q′ + G(q) | yes      |
| `computed-torque` | Joint torques: the inverse dynamics of the reference acceleration corrected by a PD on the errors     | yes      |
| `gravity-pid`     | Joint torques: gravity plus the arm's inertia times the reference acceleration and a PI-D per joint   | yes      |

**The arm controls torque.** The e-foc drives run in torque mode, the only mode the arm enables them in, and every
arm controller runs on the host (the PC
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
control period, 2·ζ·ω·T, does not depend on any inertia. The bandwidth is bounded by the loop's delay: the
telemetry's age, half a control period for the held torque, and a reading that arrives a period late under host load.
`gravity-pd` at 30 rad/s and `gravity-pid` at 20 rad/s, both at ζ = 0.7, cross over near 47 rad/s and lose stability
at about 24 ms of delay; `gravity-pid` at 15 rad/s crosses over at 37 rad/s and holds to 32 ms.

**Computed torque.** `computed-torque` is robotics-toolbox's `ComputedTorqueControl`: it asks the arm's inverse
dynamics for the torque that gives each joint the acceleration `a = q̈* + Kd·(q̇* − q̇) + Kp·(q* − q)`, so the arm's
inertia, its coupling and gravity are cancelled and every joint follows the reference as a decoupled second-order
system. The gains are in acceleration units and the same for every joint: Kp = ω²·I, Kd = 2·ζ·ω·I, from the
profile's own bandwidth and damping ratio for it. The arm model has no rotors, so the inverse dynamics it is given
adds each motor's rotor inertia times its gear ratio squared, times the joint's acceleration. Unlike `gravity-pd` it
uses the reference speed and acceleration, so it tracks a move without the lag 2·ζ/ω times the speed; like it, it
has no state and takes over without a step.

**PID with gravity feed-forward.** Neither law above has integral action, so a joint settles short of its target by
whatever torque the models leave out, the drives' Coulomb friction first: about 0.04 rad on the elbow of the
virtual arm. `gravity-pid` runs one numerical-toolbox `PidIncremental` per joint as a PI on the position error and
adds a derivative on the speed error, in acceleration units, then turns the sum into torque the way `gravity-pd` does:
`τ = J·(q̈* + u) + G(q)`, with J the inertia matrix at home and the rotors (above), `q̈*` the reference acceleration and
u the six corrections. The gains place each joint's poles at the profile's bandwidth ω: a pair with damping ζ and a
real pole for the integral, so Kp = (2·ζ + 1)·ω², Ki = ω³ (times the control period in the velocity form) and
Kd = (2·ζ + 1)·ω. The derivative takes the drives' measured speed against the reference speed rather than differencing
measured positions: a reading that arrives a period late repeats one position and then jumps, and a differenced
derivative turns that into a kick of Kd over the period; on a loaded host it tripped the virtual arm on its tracking
error. Each joint's correction is clamped to ±200 rad/s², far beyond any move's need, and the PI is clamped to the same
range: in the velocity form that clamp is the anti-windup, so a joint held against an obstacle does not wind its
integral up. `Reset` clears the six PIDs and primes each one's error history with the error at the switch, between the
reference held then and the measured joints: the proportional term sees no step, only the integral starts, so it takes
over a running move without a torque kick, and a reference step that arrives in the first period after the switch still
gets its proportional kick. Priming against the first step's own reference instead swallowed such a step: a step test
started in the period that selected `gravity-pid` rose in twice the time, on the integral alone.

The position path, where the drive closed the position loop and `pass-through` and `smoothed` shaped its setpoints,
is gone.

### Part B — Registry and selection

The arm registers its controllers once, when it is built, up to a fixed capacity of four. Ids are their
registration order. A controller is registered disabled; the arm then enables `gravity-pd`, `computed-torque` and
`gravity-pid` and selects `gravity-pd`.

| Request      | Accepted when                                                                           | Effect                                                              |
|--------------|-----------------------------------------------------------------------------------------|---------------------------------------------------------------------|
| enable *id*  | *id* is registered                                                                      | The controller may be selected                                      |
| disable *id* | *id* is registered, and it is not the active controller while the arm drives the joints | It may not be selected; if it was active, no controller is active   |
| select *id*  | *id* is enabled, and no move runs or the controller is bumpless                         | It is `Reset` from the measured joints and becomes active; reported |

Selecting the active controller again is accepted and changes nothing. In torque mode nothing but the active
controller holds the joints, so it cannot be disabled while the arm streams (`Enabling`, `Enabled`, `Moving` or `Holding`): the request is
refused with "disable the arm first". Disabled once the arm is disabled, it leaves the arm unable to stream: enabling
then aborts at the first period, until a controller is selected again. The simulator's Arm Controllers dock offers
these requests and shows each refusal ([tool-simulator.md](tool-simulator.md) Part Q), as the console's `controller`
command does ([arm-console.md](arm-console.md)).

### Part C — Streaming

While the arm streams, the active controller runs once per control period, in the control loop's setpoint phase,
after joint motion has advanced its reference:

1. The reference is the motion's sample while a move runs. Otherwise it is the last reference's positions at rest
   (zero speed and acceleration): the target of the last move, where an aborted move stopped, or the measured
   positions when streaming started.
2. The measured joints are each joint's latest reading.
3. The active controller's torques go to the safety supervisor. A set the supervisor refuses, or a period with no
   active controller, stops the streaming and aborts the running move. A set the transmit queue could not take in
   full is a missed period, not a refusal ([safety.md](safety.md) Part B): streaming goes on.

Streaming starts when the arm starts `Enabling` (the active controller is `Reset` from the measured joints) and stops
when it leaves `Enabling`, `Enabled`, `Moving` and `Holding`. So setpoints go out every period while the arm is enabled,
not only during a move: a controller holds the joints on the last reference against gravity between moves. They
already go out while the joints enable: each drive enables on its own, after its rotor has come to rest, and a joint
that runs before the others would otherwise wait without setpoints and stop on its command timeout. A drive that is
not running yet keeps the setpoint without acting on it.

### Part D — Impedance, operational space and adaptive control

Phase 5 ([#21](https://github.com/embedded-pro/e-robot-arm/issues/21)) adds four controllers, each from
robotics-toolbox's manipulator controllers over the arm model, each with its parameters in the arm profile and each
registered and enabled beside the others (Part B), `slotine-li` only when the profile allows it:

| Name                  | Law                                                                                                    | Bumpless |
|-----------------------|--------------------------------------------------------------------------------------------------------|----------|
| `joint-impedance`     | `τ = K·(q* − q) + D·(q̇* − q̇) + G(q)`: a spring and damper per joint around the reference, plus gravity | yes      |
| `cartesian-impedance` | `τ = Jᵀ·(K·e + D·ė) + G(q)`: a spring and damper on the tool's pose error e, in the base frame         | yes      |
| `operational-space`   | `τ = Jᵀ·Λ·(ẍ* + Kd·ė + Kp·e − J̇·q̇) + C + G`: the tool follows the reference as a decoupled system      | yes      |
| `slotine-li`          | `τ = Y·â − Kd·s, with s = q̇ − q̇r and â adapted as â̇ = −Γ·Yᵀ·s`: learns the arm's inertial parameters   | yes      |

**Joint impedance.** The arm yields to a push as a spring of stiffness K and a damper D per joint, centred on the
reference, with gravity compensated so it holds its own weight; nothing else is cancelled, so a move lags the
reference by about D/K times its speed. The profile names each joint's stiffness K (N·m/rad, above 0) and one damping
ratio ζ; D = 2·ζ·√(K·Jᵢᵢ), with Jᵢᵢ the joint's inertia at home with its rotor (Part A). The stiffness can be changed at
run time, between a tenth of the profile's value and the profile's value, so an operator can soften the arm without
a stiffer arm than the profile allows. It has no state: at a switch the reference is the measured pose, so its
first set is `G(q) − D·q̇`.

**Cartesian impedance.** robotics-toolbox's `ImpedanceControl` without inertia shaping: the spring and damper act on
the tool, the reference's tool pose (its forward kinematics) and twist (the Jacobian times its speeds) against the
measured ones, and Jᵀ turns the wrench into torques, so it needs no inverse and stays well-behaved near a
singularity, where it simply cannot push the tool along the lost direction. The profile names a translational
stiffness (N/m) and a rotational one (N·m/rad), each the same on the three axes, and one damping ratio; the damping
uses the tool's apparent mass at home, Λ = (J·M⁻¹·Jᵀ)⁻¹, diagonal by diagonal. The stiffnesses change at run time as
the joint's do. It needs the arm model to provide the toolbox's `JacobianProvider`: the tool Jacobian, the tool pose
and J̇·q̇.

**Operational space.** robotics-toolbox's `OperationalSpaceControl` with the full six-dimensional tool task: the task
inertia Λ, the Coriolis and gravity terms cancel the arm's dynamics as seen at the tool, so the tool follows the
reference's pose, twist and acceleration with gains Kp = ω²·I and Kd = 2·ζ·ω·I in task units, from the profile's
bandwidth and damping ratio. The task inertia is inverted with a damping σ (the profile's, 0.01 by default), so near
a singularity it trades accuracy for bounded torque. A six-joint arm with a six-dimensional task has no redundancy:
the null-space posture term is zero, and posture control waits for an arm with a seventh joint.

**Slotine–Li adaptive.** robotics-toolbox's `SlotineLiAdaptiveControl` over its `ChainInertialRegressor`: ten inertial
parameters per link, sixty in all, start at the arm model's mass properties and adapt while the arm moves, so a
payload the profile does not know is learnt rather than tracked with an error. The sliding variable is
`s = q̇ − q̇r` with `q̇r = q̇* − Λ·(q − q*)`. The profile names Λ (1/s), Kd (from a bandwidth, as `gravity-pd`'s) and the
adaptation gain Γ, and the controller is offered only when the profile's `experimental` flag is on: adaptation
with too large a Γ, or under the drives' unmodelled friction, can drift. `Reset` keeps the adapted parameters, so a
switch away and back does not lose what was learnt; the console can set them back to the model's.

**Gains away from home.** `gravity-pd` and `gravity-pid` size their gains from the inertia at home (Part A), and at
some poses the arm's coupled modes are then lightly damped: on the simulated arm at 0.1, 0.5, 0.6, 0.2, 1.0, 0 rad the
base and wrist joints 1, 4 and 6 swing together by about ±6 mrad with a period near 1.2 s, decaying over seconds. The
controllers that use the mass matrix at the measured pose (`computed-torque`, `operational-space`) do not have it.
Whether the joint-space laws recompute J at the measured pose every period, at the cost of a mass matrix per period,
is open question 3.

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

| Entity            | Field                 | Type / Unit     | Range        | Notes                                         |
|-------------------|-----------------------|-----------------|--------------|-----------------------------------------------|
| Measured joints   | positions             | radians         | finite       | Latest reading per joint                      |
| Measured joints   | speeds                | radians/second  | finite       | Latest reading per joint                      |
| Setpoint set      | positions             | radians         | joint limits | The reference, for the trace and the check    |
| Setpoint set      | torques               | N·m, joint side | finite       | Sent to the joints                            |
| `gravity-pd`      | bandwidth             | rad/s           | above 0      | 30 by default; Kp = ω²·J                      |
| `gravity-pd`      | damping ratio         | 1               | above 0      | 0.7 by default; Kd = 2·ζ·ω·J                  |
| `computed-torque` | bandwidth             | rad/s           | above 0      | 30 by default; Kp = ω²·I                      |
| `computed-torque` | damping ratio         | 1               | above 0      | 0.7 by default; Kd = 2·ζ·ω·I                  |
| `gravity-pid`     | bandwidth             | rad/s           | above 0      | 20 by default; Kp = (2ζ + 1)·ω², Ki = ω³      |
| `gravity-pid`     | damping ratio         | 1               | above 0      | 0.7 by default; Kd = (2ζ + 1)·ω, on the speed |
| `gravity-pid`     | output limit          | rad/s²          | ±200         | Per joint; the integral's anti-windup         |
| Reference         | speeds, accelerations | rad/s, rad/s²   | finite       | Zero while holding                            |
| Registry entry    | id                    | integer         | 0–7          | Registration order                            |
| Registry entry    | name                  | text            | unique       | The names of Parts A and D                    |
| Registry entry    | enabled               | flag            |              | Registered disabled                           |

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
    participant C as gravity-pd
    U->>A: select gravity-pd (move running)
    A->>C: reset (measured joints)
    A-->>U: active controller: gravity-pd
    Note over A,C: next period's torques start from the measured joints
```

---

## Constraints & Limitations

| Constraint | Value / Description                                                                      |
|------------|------------------------------------------------------------------------------------------|
| Capacity   | Eight controllers                                                                        |
| Bumpless   | From the measured joints: the setpoints move by at most the tracking error at the switch |
| Traffic    | One setpoint set per control period while the arm is enabled                             |
| Memory     | No heap; every controller's state is fixed-size                                          |

---

## Open Questions

| # | Question                                                | Options                                            | Status                       |
|---|---------------------------------------------------------|----------------------------------------------------|------------------------------|
| 1 | Whether the profile stores the enabled and active ones  | Profile section; console and simulator choice only | open: with phase 5           |
| 2 | Whether a switch to a non-bumpless controller may blend | Refuse while moving; ramp from the old set         | decided: refuse while moving |
| 3 | Gains of the joint-space laws away from home            | Inertia at home; mass matrix at the measured pose  | open (Part D)                |
