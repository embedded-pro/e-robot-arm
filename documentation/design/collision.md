---
title: "Collision Observer Design"
type: design
status: draft
version: 0.4.0
component: "collision"
date: 2026-11-17
---

| Field     | Value                     |
|-----------|---------------------------|
| Title     | Collision Observer Design |
| Type      | design                    |
| Status    | draft                     |
| Version   | 0.4.0                     |
| Component | collision                 |
| Date      | 2026-11-17                |

> Estimates the external torque on each joint from the arm's dynamics, the measured joints and the torques the arm
> commanded, so the safety supervisor can stop the arm on a collision. Behaviour only; diagrams are Mermaid.

---

## Responsibilities

**Is responsible for:**

- Estimating, once per control step while the arm drives its joints, the torque on each joint that the arm's
  dynamics do not explain: an obstacle, a person, a payload the profile does not name.
- Starting from no external torque whenever the arm starts driving its joints again.

**Is NOT responsible for:**

- Deciding whether an estimate is a collision, or stopping the arm: the safety supervisor compares each joint's
  estimate with its collision limit ([safety.md](safety.md) Part F).
- Telling where on a link the arm was hit, or reacting with compliance: a collision trips the arm.
- Modelling friction: the drives' friction shows in the estimate, and the collision limit sits above it.

---

## Component Details

### Part A — The momentum observer

The arm's generalised momentum is its inertia matrix times the joint speeds. Without an external torque it changes
only by the commanded torque, gravity and the Coriolis terms the dynamics give. The observer is robotics-toolbox's
momentum observer: it integrates that expected change, compares the integral with the momentum measured now, and
the difference, scaled by a gain, is the external torque. The estimate follows the true external torque as a
first-order lag with the gain as its bandwidth: 25 /s by default. Over a 10 ms period each step closes a quarter of
the gap, so a step of external torque shows at 25 % after one period and at 68 % after four: a 30 N·m collision
crosses a 10 N·m limit in two periods. It needs no joint accelerations, so measured speeds are enough, and no
differentiation, so it does not amplify noise. The gain also scales every timing error in the torque the observer
integrates: at 50 /s a reading that arrives a few milliseconds off made the reference step of a tuning run read
10 N·m on the shoulder of the virtual arm, at 25 /s it reads below 2 N·m.

The dynamics are the arm model's ([arm-model.md](arm-model.md)): its links' mass, centre of mass and inertia,
gravity, and each joint's reflected rotor inertia, the motor's inertia times the gear ratio squared, as the link's
armature. The rotor inertia matters: on a 50:1 gear it is of the order of the links' own inertia, and an observer
without it reads every acceleration as an external torque.

### Part B — When it runs

The arm steps the observer after the arm controllers, once per control step:

- While the controllers stream setpoints, it updates with the latest measured positions and speeds and the torque
  that acted since the previous reading. The control loop requests telemetry half a period before it sends the
  setpoints ([control-loop.md](control-loop.md)), so between two readings the set commanded two steps before acted
  for the first half and the set commanded the step before for the second: the observer integrates their mean.
- It updates only once every joint has reported since its previous update. A step in which a joint's reading has not
  arrived keeps the previous estimate; the next update integrates over the time between the telemetry requests the two
  readings answered, with the mean of the torque that acted in the periods between. A host that runs late sends its
  requests further apart than the period; integrating over the nominal period then misses the torque of the extra
  time, about 5 N·m on the virtual arm's accelerating shoulder for 5 ms. Taking a held-over reading as new integrates a period's torque against a
  momentum that did not move: on a busy host one missed reading of a shoulder that accelerates read 11 N·m.
- The first streaming step after any step without streaming resets it to the momentum measured then, and reports no
  estimate for that step.
- A step without streaming reports no estimate.

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Observing: streaming starts, reset to the measured momentum
    Observing --> Observing: streaming step, update and report
    Observing --> Idle: a step without streaming
```

The estimate is reported in joint space, in N·m, with the sign of the joint's torque; the arm host shows it in its
snapshot and trace.

---

## Interfaces

### Provided

| Interface | Purpose                                                    | Contract                                        |
|-----------|------------------------------------------------------------|-------------------------------------------------|
| Reset     | Start from no external torque at the measured joints       | Called when streaming starts                    |
| Update    | The external torque on each joint after one control period | Measured joints, the previous commanded torques |

### Required

| Interface | Purpose                                        | Contract                                   |
|-----------|------------------------------------------------|--------------------------------------------|
| Arm model | Links, gravity, joint-space conversions        | Built from the profile's geometry and mass |
| Rotors    | Each joint's reflected rotor inertia, in kg·m² | From the profile's motors and gear ratios  |

---

## Data Model

| Entity   | Field | Type / Unit | Range   | Notes                                      |
|----------|-------|-------------|---------|--------------------------------------------|
| Observer | gain  | 1/s         | above 0 | The estimate's bandwidth; 25 /s by default |
| Estimate | joint | N·m         | finite  | One per joint, joint space                 |

---

## Constraints & Limitations

- The estimate carries every torque the model misses: Coulomb friction while a joint moves, an unmodelled payload,
  wrong link masses. On the virtual arm it stays below 4.6 N·m through the reference move, on the shoulder; the
  collision limit sits above that.
- A reading that arrives within the period but late in it still shows the joints a little earlier than the torque
  assumes; the error is a fraction of the gain times one period's momentum change and the next reading corrects it.
- The observer takes the commanded torque as applied. A drive that cannot deliver it, at its current limit or
  behind its current loop, leaves the difference in the estimate: a 0.1 rad reference step on the virtual arm's
  elbow reads up to 5.6 N·m on the shoulder. The scenarios step 0.05 rad, below 2 N·m, to keep a margin to the
  limit until the observer knows the drives' limits (open question 2).
- The observer is heap-free and runs once per control step; a mass matrix, a Coriolis matrix and a gravity vector
  per step.

---

## Open Questions

| # | Question                                                                       | Options                                                                              | Status |
|---|--------------------------------------------------------------------------------|--------------------------------------------------------------------------------------|--------|
| 1 | Whether the observer models the drives' friction, so the limits can come down  | No friction model, Coulomb and viscous per joint                                     | open   |
| 2 | Whether the observer clamps the commanded torque to what the drive can deliver | Commanded torque, clamped to the motor's current limit, the drive's measured current | open   |
