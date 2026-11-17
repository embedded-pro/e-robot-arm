---
title: "Joint Motion Design"
type: design
status: draft
version: 0.8.0
component: "motion"
date: 2026-11-14
---

| Field     | Value               |
|-----------|---------------------|
| Title     | Joint Motion Design |
| Type      | design              |
| Status    | draft               |
| Version   | 0.8.0               |
| Component | motion              |
| Date      | 2026-11-14          |

> Moving the six joints together from where they are to a target, in joint space. Behaviour only; diagrams are
> Mermaid.

---

## Responsibilities

**Is responsible for:**

- Planning a point-to-point move of all six joints that starts and ends at rest, within each joint's speed and
  acceleration limits.
- Synchronising the joints so that they start together and arrive together.
- Sampling the planned positions, speeds and accelerations once per control period, and the target at rest last,
  as the reference the arm controllers follow ([arm-controllers.md](arm-controllers.md)).
- A reference step: the reference jumps to a target close to the joints at once, for tuning the arm controllers.
- Reporting when the move is done, or when it was aborted.

**Is NOT responsible for:**

- Sending setpoints: the active arm controller turns the reference into setpoint sets, and the safety supervisor
  guards them; a set it refuses aborts the move. Joint motion only refuses a target outside the position limits up
  front.
- Requesting telemetry: the arm's control loop paces the requests.
- Paths through waypoints, Cartesian moves, jerk limits and dynamics: planned with the arm model.

---

## Component Details

### Part A — Planning

A move starts from each joint's latest reported position and ends at the requested target. Each joint is first
planned on its own as a trapezoidal velocity profile: accelerate at its acceleration limit, cruise at its speed
limit if the distance allows it, decelerate at the same limit. The slowest joint sets the move's duration.

Every other joint is then re-planned to take exactly that duration, at its own acceleration limit, which lowers
its cruise speed. A joint that does not move stays where it is. All joints therefore start and stop at the same
instants, and no joint exceeds its limits.

A move is refused if a target is not finite, lies outside its joint's position limits, another move is still
running, or a joint's latest position is stale: it answered neither the latest telemetry request nor the one before
it ([joint-bus.md](joint-bus.md) Part E), so the move would start from where the joint no longer is. Each joint moves monotonically from its start to its target, so a target within the limits keeps the
whole move within them; the safety supervisor still checks every setpoint set.

### Part B — Sampling

Joint motion has no clock of its own: the control loop runs its step once per control period, in the setpoint
phase. While a move is running, each step advances the move by one period and samples each joint's position, speed
and acceleration at that time, measured from the start of the move, from the same trapezoid the move was planned
with: that is the period's reference, which the arm controllers turn into a
setpoint set right after. With no move running a step has no reference. The first sample is taken at the first
step after the start. The sample at or after the move's end is the target itself, at rest (zero speed and
acceleration); it is given once and the move is done.

### Part C — Reference step

A step is a move without a trajectory: the reference jumps from where the joints are to the target, at rest, so the
arm controllers see a true step and its response shows their bandwidth and damping (the tuning views of
[tool-simulator.md](tool-simulator.md)). The first sample after a step is the target at rest, and the step is done
at once; the arm controllers then hold the target. A step is refused for every reason a move is (Part A), and also
when any joint's target lies more than 0.2 rad from its latest position: a larger jump asks the controllers for more
torque than a test needs, and a joint that should travel further is moved, not stepped.

### Part D — Aborting

If the arm cannot send the period's set (the safety supervisor refuses it, or no arm controller is active), the
arm aborts the move. Aborting a move, by request or so, stops the sampling; the arm controllers hold the last
reference, so the joints stay where the move left them.

---

## Interfaces

### Provided

| Interface           | Purpose                                                     | Contract                                                 |
|---------------------|-------------------------------------------------------------|----------------------------------------------------------|
| Joint motion        | Move or step to a target, abort, whether moving             | One move or step at a time; refused targets send nothing |
| Joint motion events | Move done, move aborted                                     | Exactly one of the two per accepted move                 |
| Reference           | Position, speed and acceleration per step while a move runs | None without a move; the target at rest is the last      |

### Required

| Interface | Purpose                     | Contract                                 |
|-----------|-----------------------------|------------------------------------------|
| Joint bus | Latest positions            | The safety supervisor                    |
| Step      | One step per control period | Run by the control loop's setpoint phase |

---

## Data Model

| Entity       | Field            | Type / Unit     | Range         | Notes                                    |
|--------------|------------------|-----------------|---------------|------------------------------------------|
| Joint limits | position         | radians         | lower < upper | One range per joint, as the supervisor's |
| Joint limits | maximum speed    | radians/second  | > 0           | One per joint                            |
| Joint limits | max acceleration | radians/second² | > 0           | One per joint                            |
| Motion       | control period   | duration        | > 0           | Time between two setpoint sets           |
| Reference    | position         | radians         | joint limits  | Per joint, from the move's trapezoid     |
| Reference    | speed            | radians/second  | finite        | Per joint; zero at rest                  |
| Reference    | acceleration     | radians/second² | finite        | Per joint; zero at rest                  |
| Step         | size             | radians         | at most 0.2   | Per joint, from its latest position      |

---

## State Machine

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Moving : move or step [target finite]
    Moving --> Idle : target sent / done
    Moving --> Idle : set not sent / aborted
    Moving --> Idle : abort / aborted
```

---

## Sequence Diagrams

```mermaid
sequenceDiagram
    participant C as Controller
    participant M as Joint motion
    participant B as Joint bus (safety supervisor)
    participant A as Arm controllers
    C->>M: move to target
    M->>B: latest positions
    loop each control loop step
        M->>A: sampled reference
    end
    M->>A: target
    M-->>C: done
```

---

## Constraints & Limitations

| Constraint  | Value / Description                                                                 |
|-------------|-------------------------------------------------------------------------------------|
| Profile     | Trapezoidal velocity: acceleration steps at the start and end of each phase         |
| Start state | At rest, at the latest reported positions; a move does not blend into a running one |
| Memory      | No heap; all state is fixed-size                                                    |

---

## Open Questions

| # | Question                                        | Options                                            | Status |
|---|-------------------------------------------------|----------------------------------------------------|--------|
| 1 | Whether to limit jerk                           | Trapezoidal, S-curve per joint (numerical-toolbox) | open   |
| 2 | Whether a new target may replace a running move | Refuse, re-plan from the current state             | open   |
