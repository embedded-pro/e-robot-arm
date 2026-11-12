---
title: "Control Loop Design"
type: design
status: draft
version: 0.1.0
component: "control_loop"
date: 2026-11-12
---

| Field     | Value               |
|-----------|---------------------|
| Title     | Control Loop Design |
| Type      | design              |
| Status    | draft               |
| Version   | 0.1.0               |
| Component | control_loop        |
| Date      | 2026-11-12          |

> The arm's fixed-period cycle: when telemetry is requested and when setpoints go out. Behaviour only; diagrams
> are Mermaid.

---

## Responsibilities

**Is responsible for:**

- Running the arm at a fixed control period once started, until stopped.
- Requesting telemetry from all six joints at the start of every period.
- Running the setpoint step (joint motion today) half a period later, once the telemetry has had time to arrive.

**Is NOT responsible for:**

- What the setpoint step sends: joint motion plans and samples the positions.
- Detecting silent or faulted joints: the joint bus counts missed requests and the safety supervisor reacts.
- Starting, stopping or homing the joints themselves.

---

## Component Details

### Part A — Two phases per period

Each period has two phases, half a period apart:

1. **Telemetry**: one telemetry request to all six joints. The joints answer with their status and motion
   telemetry, which updates the readings the next phase plans from.
2. **Setpoints**: the setpoint step runs once. Joint motion samples its move and sends one setpoint set, or does
   nothing when no move is running.

Starting the loop requests telemetry at once; the first setpoint step follows half a period later. Stopping the
loop ends both phases; a phase that has not run yet does not run.

### Part B — Why the phases are apart

A telemetry request and a setpoint set are six frames each, and can-lite's transmit queue holds eight, so the two
bursts must not be queued together. Half a period also gives the joints time to answer before positions are
sampled: the request and its twelve answers take about 2.1 ms of a 1 Mbit/s bus, so the period is at least
5 ms (200 Hz).

---

## Interfaces

### Provided

| Interface    | Purpose                  | Contract                                                       |
|--------------|--------------------------|----------------------------------------------------------------|
| Control loop | Start and stop the cycle | Starting a running loop or stopping a stopped one does nothing |

### Required

| Interface     | Purpose                                    | Contract                                        |
|---------------|--------------------------------------------|-------------------------------------------------|
| Joint bus     | Telemetry requests                         | The safety supervisor, in front of the real bus |
| Setpoint step | Runs once per period in the setpoint phase | Joint motion's step                             |
| Timer         | Half the control period                    | The event loop's timer service                  |

---

## Data Model

| Entity       | Field          | Type / Unit | Range  | Notes                                   |
|--------------|----------------|-------------|--------|-----------------------------------------|
| Control loop | control period | duration    | ≥ 5 ms | Telemetry and setpoint phases each half |

---

## State Machine

```mermaid
stateDiagram-v2
    [*] --> Stopped
    Stopped --> Telemetry : start / request telemetry
    Telemetry --> Setpoints : half a period / setpoint step
    Setpoints --> Telemetry : half a period / request telemetry
    Telemetry --> Stopped : stop
    Setpoints --> Stopped : stop
```

---

## Sequence Diagrams

```mermaid
sequenceDiagram
    participant L as Control loop
    participant B as Joint bus (safety supervisor)
    participant M as Joint motion
    participant N as e-foc node (x6)
    L->>B: request telemetry
    B->>N: request telemetry
    N-->>B: status and motion telemetry
    Note over L: half a period later
    L->>M: setpoint step
    M->>B: setpoint set
    B->>N: position setpoints
```

---

## Constraints & Limitations

| Constraint | Value / Description                                                                                |
|------------|----------------------------------------------------------------------------------------------------|
| Bus load   | About 24 frames per period (6 requests, 12 answers, 6 setpoints): about 57 % of 1 Mbit/s at 200 Hz |
| Timing     | Event-loop timers: on a PC the phases jitter with the operating system's scheduling                |
| Memory     | No heap; all state is fixed-size                                                                   |

---

## Open Questions

| # | Question                                      | Options                             | Status |
|---|-----------------------------------------------|-------------------------------------|--------|
| 1 | Whether to measure and report period overruns | Report a late phase, count overruns | open   |
