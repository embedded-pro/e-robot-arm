---
title: "Virtual Arm Design"
type: design
status: draft
version: 0.3.0
component: "virtual-arm"
date: 2026-11-15
---

| Field     | Value              |
|-----------|--------------------|
| Title     | Virtual Arm Design |
| Type      | design             |
| Status    | draft              |
| Version   | 0.3.0              |
| Component | virtual-arm        |
| Date      | 2026-11-15         |

> The arm run against six real e-foc firmware instances instead of ideal simulated joints: e-foc's headless
> virtual motor, one process per joint, on one virtual CAN bus, each loaded by what its link and the links beyond
> it put on its shaft (phase 7, [#23](https://github.com/embedded-pro/e-robot-arm/issues/23)). Behaviour only;
> diagrams are Mermaid. The virtual motor itself is e-foc's `virtual-motor.md`.

---

## Responsibilities

**Is responsible for:**

- Starting six e-foc virtual motors from an arm profile, one per joint, each with the motor the profile names, on
  one virtual CAN bus at the profile's node ids, and stopping them.
- Coupling each motor to the arm's dynamics: the torque its link and the links beyond it apply, through its
  transmission, fed to the motor's plant as external shaft torque while the arm moves.
- Faults on demand: a silent joint, a collision torque on a joint, a joint powered off.
- Running headless, so the console, the simulator, the scenarios and CI start the same virtual arm.

**Is NOT responsible for:**

- Controlling the arm: the arm reaches the virtual motors through `virtual:<bus>` like any other CAN bus, and
  nothing in `core/` knows the motors are virtual.
- Simulating a motor: each virtual motor runs e-foc's shipping application on e-foc's plant; the virtual arm only
  chooses the plant and pushes torque on its shaft.
- Verifying e-foc: a virtual motor is a bench, not a test oracle (`virtual-motor.md` Part I).

---

## Component Details

### Part A — Process layout

The virtual arm is a host-only tool, `e-robot-arm-virtual-arm` (`tools/virtual_arm/`). It reads an arm profile,
opens a plant-port listener on a loopback port, and starts six `e-foc-virtual-motor` processes, each told to
connect to it. The first motor hosts the virtual CAN hub for the bus; the arm joins the same bus as
`virtual:<bus>`.

```mermaid
graph LR
    subgraph HOST["Development machine or CI runner"]
        ARM["Arm (console, simulator or scenario)"]
        VA["Virtual arm: launcher and coupling"]
        HUB["Virtual CAN hub: bus 'arm'"]
        M1["e-foc virtual motor, joint 1"]
        M6["e-foc virtual motor, joint 6"]
    end
    ARM <-->|"CAN: virtual:arm"| HUB
    M1 <-->|"CAN"| HUB
    M6 <-->|"CAN"| HUB
    VA -.->|"starts, stops"| M1
    VA -.->|"starts, stops"| M6
    M1 <-->|"plant port"| VA
    M6 <-->|"plant port"| VA
```

Each motor is started with:

| Start option | Value                                                                                            |
|--------------|--------------------------------------------------------------------------------------------------|
| Bus          | The virtual arm's bus name, `arm` unless told otherwise                                          |
| Node         | The joint's node id minus 1: a virtual motor's node is its configured base, 1, plus its board ID |
| Plant        | The joint's motor (Part B)                                                                       |
| Memory       | `joint<n>.mem` in the virtual arm's working directory                                            |
| Start from   | Calibrated, so every joint reaches *Ready* without an identification run                         |
| Seed         | The joint number, so a run replays exactly                                                       |
| Host the bus | The first motor only                                                                             |
| Plant port   | The virtual arm's listener                                                                       |

A node id outside 1 to 256 cannot be reached this way and the virtual arm refuses the profile. The working
directory is one per virtual arm run, so two runs never share memory files.

**Start.** The virtual hub's loopback port must be free or held by a running hub first: a hub that closed less than a
minute ago leaves it held by closed connections, and the virtual arm waits up to 70 s for it before it refuses to
start. It is up once all six motors have connected to the plant port, introduced themselves (Hello, protocol version
2) and sent their first status, which a motor does once it has joined the bus, within 10 s; otherwise it stops the
ones it started and reports which joint did not come up. Each motor's output goes to `joint<n>.log` in the working
directory. **Stop.** It ends every motor it started, the first motor last so that the hub closes after its members,
which leaves the bus silent as a powered-off arm; a motor that ends without being asked is reported with the last
lines of its debug log. Each motor runs in a process group of its
own, so Ctrl-C reaches only the virtual arm, which then stops the motors; a motor whose virtual arm dies is ended
with it, since an e-foc virtual motor otherwise outlives its plant port.

### Part B — Motor per joint

A virtual motor's motor is its plant, fixed when it starts (`virtual-motor.md` Part E). The profile's motor per
joint ([arm-profile.md](arm-profile.md)) chooses it:

| Profile motor                           | Plant                                                          |
|-----------------------------------------|----------------------------------------------------------------|
| None                                    | `teknic`, e-foc's nominal motor                                |
| A reference motor (`teknic`, `anaheim`) | The e-foc plant preset of the same name                        |
| Custom values                           | Refused: e-foc reads custom plants only in its own binary form |

Changing a joint's motor in the profile and restarting that joint is enough: the virtual arm stops the motor, and
starts it again with the new plant and a fresh calibrated memory. Custom values wait for e-foc to read a plant
description as text (open question 1).

### Part C — Coupling to the arm's dynamics

Each motor's plant has its own rotor; nothing on its shaft but what the plant port pushes. The virtual arm closes
that loop with the arm model ([arm-model.md](arm-model.md) Part F):

1. Each motor streams its plant signals through the plant port (`virtual-motor.md` Part F), a window every 10
   control periods (2 kHz). From each window the virtual arm takes the last period's true rotor angle and speed,
   unwraps the angle across turns, and converts both to joint space through the joint's transmission
   ([joint-bus.md](joint-bus.md) Part F).
2. Every millisecond it computes, at the latest joint positions q and speeds q̇ of all six joints, the torque each
   joint must supply to hold the links against gravity and their speed-dependent coupling: the arm model's inverse
   dynamics at zero acceleration, τ = M(q)·0 + C(q, q̇)·q̇ + G(q).
3. The links push back with −τ. Each joint's share goes to its motor's shaft through the transmission,
   motor torque = −τ / (direction · ratio), as a constant external torque (`virtual-motor.md` Part E), which the
   motor applies from its next control period.

```mermaid
sequenceDiagram
    participant M as Virtual motors (x6)
    participant V as Virtual arm
    participant A as Arm model
    M-->>V: plant windows (rotor angle, speed), 2 kHz each
    loop every millisecond
        V->>V: unwrap, to joint space
        V->>A: inverse dynamics at q, q̇, zero acceleration
        A-->>V: joint torques τ
        V->>M: external torque −τ / (direction · ratio) per motor
    end
```

**What the coupling captures, and what it leaves out.** Gravity, the full load of the links and the payload on
every joint, and the centrifugal and Coriolis torques between joints, all from the measured state of all six.
It leaves out the links' inertia: a motor accelerates only its own rotor, so a joint driven hard accelerates
faster than the real arm would and the coupling between one joint's acceleration and another's torque is absent.
Feeding the inertia through measured accelerations would close an algebraic loop one window late; the faithful
way is to add each link's reflected inertia to its motor's plant, which needs custom plants (open question 2).
The torque reaches a motor at most 1 ms plus one window late; at the arm's speeds that lag is below what the
scenarios measure.

### Part D — Faults on demand

| Fault             | How                                                                                          | What the arm sees                                      |
|-------------------|----------------------------------------------------------------------------------------------|--------------------------------------------------------|
| Silent joint      | Pause the motor through its plant port; resume to bring it back                              | The node stops answering (`virtual-motor.md` Part C)   |
| Collision torque  | An extra external torque on a joint, added to the coupling, as a step held until cleared     | Following error, then the node's own fault if it lasts |
| Joint powered off | End the motor's process; restarting it brings the joint back calibrated from its memory file | The node goes silent and comes back unpowered          |

A scenario or the console asks the virtual arm for them; the arm only sees their effect on the bus.

### Part E — Who starts it

| User                | How                                                                                             |
|---------------------|-------------------------------------------------------------------------------------------------|
| Console             | `--virtual-arm <profile>` starts the virtual arm, then the console's own arm on `virtual:<bus>` |
| Simulator           | A *Virtual e-foc motors* target beside *Simulated joints* (phase 8d)                            |
| Scenarios           | A `@virtual` fixture starts it before a scenario and stops it after                             |
| By hand or a script | `e-robot-arm-virtual-arm --profile <file> [--bus <name>]`, ended with Ctrl-C                    |

It finds `e-foc-virtual-motor` on the path, or where `--engine <path>` says.

### Part F — The e-foc virtual motor, built or downloaded

e-robot-arm does not build e-foc. The virtual motor comes from e-foc's release: its Linux tools archive carries
`e-foc-virtual-motor`, and CI downloads it at a pinned release and checks its protocol version on Hello. A
developer installs the same release, or builds e-foc and points `--engine` at it.

| Option                                | For                                         | Against                                                          |
|---------------------------------------|---------------------------------------------|------------------------------------------------------------------|
| Download e-foc's release (chosen)     | Seconds in CI; the binary e-foc's users run | Follows e-foc's releases, not its `main`                         |
| Build from an `infra/e-foc` submodule | Any e-foc revision                          | e-foc and its submodules, minutes of build per run, one more pin |

The plant port is a small framed protocol on a loopback socket. The virtual arm implements the part it uses
(Hello, Status, Changes, Refused, Log line and Window from the motor; Pause, Resume, External torque, Clear all
and Stream to it) from `virtual-motor.md` Part F, and refuses a motor that does not speak version 2.

### Part G — Scenarios

`@virtual` scenarios run the arm's features against the virtual arm, reusing the steps of the simulated-joint
scenarios where the wording holds, and add:

- tracking error under gravity: the largest difference between commanded and measured position during a move
  that lifts the shoulder, reported as a `[METRIC]` line and held to a limit pinned from it;
- a collision: a collision torque on a moving joint stops the arm;
- a silent joint: a paused motor trips the arm as a silent simulated joint does.

They run in real time, so a CI job of their own runs them: it downloads the e-foc release, builds the host
targets, and reports the metrics. Tracking error with gravity compensation waits for phase 5's controller, and a
start-up through calibration waits for the lifecycle to drive one (open question 3).

---

## Interfaces

### Provided

| Interface        | Purpose                                           | Contract                                       |
|------------------|---------------------------------------------------|------------------------------------------------|
| Virtual arm tool | Six virtual motors from a profile, on one bus     | Up within 10 s or stopped and reported; Part A |
| Coupling         | Arm-dynamics torque on every motor's shaft        | Every millisecond while up; Part C             |
| Faults           | Silent joint, collision torque, joint powered off | Part D; each reversible                        |

### Required

| Interface                   | Purpose                                      | Contract                                                                                 |
|-----------------------------|----------------------------------------------|------------------------------------------------------------------------------------------|
| `e-foc-virtual-motor`       | One joint's drive                            | e-foc's release; start options and plant port per `virtual-motor.md`, protocol version 2 |
| Arm profile                 | Nodes, transmissions, motors, geometry, mass | [arm-profile.md](arm-profile.md)                                                         |
| Arm model                   | Inverse dynamics                             | [arm-model.md](arm-model.md)                                                             |
| can-lite `virtual:` adapter | The arm's side of the bus                    | Unchanged                                                                                |

---

## Data Model

| Entity      | Field       | Type / Unit | Range                     | Notes                                              |
|-------------|-------------|-------------|---------------------------|----------------------------------------------------|
| Virtual arm | bus         | name        | letters, digits, `-`, `_` | `arm` by default                                   |
| Virtual arm | plant port  | TCP port    | loopback                  | Chosen by the system unless given                  |
| Joint       | node        | integer     | 1–256                     | From the profile; the motor's board ID is node − 1 |
| Joint       | plant       | preset name | `teknic`, `anaheim`       | From the profile's motor (Part B)                  |
| Coupling    | period      | duration    | 1 ms                      | Torques computed from the latest windows           |
| Coupling    | joint state | rad, rad/s  | unwrapped                 | From each motor's true rotor angle and speed       |
| Collision   | torque      | N·m         | finite                    | Joint-side; added to the coupling until cleared    |

---

## State Machine

```mermaid
stateDiagram-v2
    [*] --> Starting : profile accepted
    Starting --> Up : six motors said Hello
    Starting --> Stopped : a motor did not come up within 10 s / stop the others, report it
    Up --> Up : coupling every millisecond
    Up --> Up : fault injected or cleared
    Up --> Restarting : a joint's motor changed
    Restarting --> Up : that motor said Hello again
    Up --> Stopped : stop / end every motor
    Up --> Stopped : a motor ended unasked / report it, end the others
    Stopped --> [*]
```

---

## Constraints & Limitations

| Constraint | Value / Description                                                                                                                                                                                                                                                                    |
|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Coupling   | Gravity, payload and speed-dependent terms; not the links' inertia (Part C)                                                                                                                                                                                                            |
| Motors     | e-foc's plant presets only until e-foc reads text plant descriptions                                                                                                                                                                                                                   |
| Time       | Real time: virtual motors run on the wall clock                                                                                                                                                                                                                                        |
| Platform   | Linux first, as CI; Windows when e-foc's Windows archive is wired the same way                                                                                                                                                                                                         |
| Network    | Loopback only                                                                                                                                                                                                                                                                          |
| Bus load   | One telemetry request and two answers per joint per period: 1800 frames/s at 100 Hz. can-lite's hub must deliver every frame whose transmission time has passed at once; one per pass of its host's event loop (about 500 frames/s in an e-foc virtual motor) leaves the joints silent |

---

## Open Questions

| # | Question                         | Options                                                                     | Status |
|---|----------------------------------|-----------------------------------------------------------------------------|--------|
| 1 | Custom motors on a virtual joint | e-foc reads a text plant description; e-robot-arm writes e-foc's binary one | open   |
| 2 | The links' inertia               | Reflected inertia added to each plant (needs 1); measured accelerations     | open   |
| 3 | A start-up through calibration   | The lifecycle drives calibration; start factory-fresh and calibrate by hand | open   |
| 4 | Where the plant-port codec lives | In e-robot-arm (chosen for now); a library e-foc publishes                  | open   |
