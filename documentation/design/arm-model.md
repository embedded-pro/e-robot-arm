---
title: "Arm Model Design"
type: design
status: draft
version: 0.4.1
component: "arm_model"
date: 2026-11-26
---

| Field     | Value            |
|-----------|------------------|
| Title     | Arm Model Design |
| Type      | design           |
| Status    | draft            |
| Version   | 0.4.1            |
| Component | arm_model        |
| Date      | 2026-11-26       |

> The arm's geometry and mass, and what follows from them: the tool pose of a set of joint positions, the joint
> positions of a tool pose, the Jacobian and the singularities, the gravity torque, the joint torques of a motion and
> the mass matrix. The mathematics is in [arm-kinematics.md](../theory/arm-kinematics.md) and
> [arm-dynamics.md](../theory/arm-dynamics.md). Behaviour only; diagrams are Mermaid.

---

## Responsibilities

- Holding the arm's geometry: the seven OPW lengths, a sign and an offset per joint, the tool transform and the base
  mount.
- Forward kinematics: the tool pose of six joint positions, in the base frame.
- Inverse kinematics: the joint positions of a tool pose, within the joint limits and closest to the current joints,
  or the reason there are none.
- The geometric Jacobian, the manipulability and the singularity the arm is near.
- Holding each link's mass properties and the payload's.
- Dynamics: the gravity torque, the inverse dynamics (joint torques of positions, speeds and accelerations) and the
  mass matrix, in joint space.
- Staying heap-free, with a fixed cost per call, so it can run in the control loop.

---

## Component Details

### Part A — Geometry

| Item                      | Unit   | Meaning                                                                  |
|---------------------------|--------|--------------------------------------------------------------------------|
| a1, a2, b, c1, c2, c3, c4 | m      | The OPW lengths ([arm-kinematics.md](../theory/arm-kinematics.md))       |
| Joint signs               | ±1     | Direction of each joint in the model                                     |
| Joint offsets             | rad    | The model angle at the joint's zero                                      |
| Tool                      | m, rad | The tool centre point relative to the flange: position, roll, pitch, yaw |
| Base                      | m, rad | The arm's base relative to the world: position, roll, pitch, yaw         |

Orientations are roll, pitch and yaw about fixed x, y and z: $R = R_z(\text{yaw})\,R_y(\text{pitch})\,R_x(\text{roll})$.
The joint positions are joint space, after the transmission ([joint-bus.md](joint-bus.md)), so a gear ratio never
reaches the model. Without a geometry in the profile the arm uses the reference desktop arm:

| c1   | a1   | b | c2   | a2 | c3   | c4   | Signs  | Offsets | Tool, base |
|------|------|---|------|----|------|------|--------|---------|------------|
| 0.15 | 0.02 | 0 | 0.25 | 0  | 0.25 | 0.06 | all +1 | all 0   | identity   |

It is invented: about 0.56 m of reach from the shoulder, standing for a desktop arm until the real one is measured.

The signs and offsets are one affine map between joint space and the model's angles, the joint frame map. Let S be
the diagonal matrix of the signs (so S⁻¹ = S) and o the offsets. Then:

$$\theta = S\,q - o, \qquad \dot\theta = S\,\dot q, \qquad \tau_\theta = S\,\tau, \qquad J_q = J_\theta\,S, \qquad M_q = S\,M_\theta\,S.$$

The map keeps S as its diagonal, so each product costs one multiplication per entry.

### Part B — Forward kinematics, Jacobian and singularities

- **Tool pose** of six joint positions: base · flange · tool.
- **Jacobian**: 6 × 6, linear rows first, in the base frame, at the tool centre point.
- **Task Jacobian**: the same Jacobian, tool pose and bias acceleration J̇·q̇ in the form that robotics-toolbox's
  manipulator controllers read. It is in the dynamics' coordinates (each joint's sign and offset applied). Thus the
  toolbox's manipulator controllers take the model's Jacobian with its mass matrix and gravity
  ([arm-controllers.md](arm-controllers.md) Part D).
- **Manipulability**: |det J|.
- **Singularities**: wrist, elbow and shoulder, each flagged on its own by the thresholds of the theory.
- **Link frames**: the pose of each of the six links in the base frame, at its joint, as
  [arm-dynamics.md](../theory/arm-dynamics.md) defines them. Link 6 with the flange at (0, 0, c4) and the tool
  gives the tool pose. The simulator draws the arm from them ([tool-simulator.md](tool-simulator.md) Part E).

### Part C — Inverse kinematics

Given a tool pose and the current joints, the model returns the solution within the joint limits that is closest to
the current joints. Otherwise it returns one of two failures:

| Failure              | When                                                      |
|----------------------|-----------------------------------------------------------|
| Unreachable          | No solution: the target is out of reach                   |
| Outside joint limits | Solutions exist, none with all six joints in their limits |

A solution at a wrist singularity is returned with θ4 = 0, the joint pair 4 and 6 being free; the singularity flag
says so.

### Part D — Where the tools use it

- The arm host publishes the measured tool pose and the singularity flags in each snapshot.
- The console prints them (`pose`), and converts on request without moving the arm ([arm-console.md](arm-console.md)).
  The command `fk q1 … q6` prints a tool pose. The command `ik x y z roll pitch yaw` prints the joints or the failure.

### Part F — Mass properties and dynamics

Each link has a mass, a centre of mass and an inertia tensor about it, in the link's frame
([arm-dynamics.md](../theory/arm-dynamics.md)). The payload has the same, relative to the flange, and is merged into
the last link. Without mass properties in the profile, the arm uses the reference desktop arm's and no payload.
They are invented like its geometry (aluminium tubes and small geared motors, about 3.7 kg moving):

| Link | Mass kg | Centre m (x, y, z) | Ixx, Iyy, Izz kg·m²       | Body                      |
|------|---------|--------------------|---------------------------|---------------------------|
| 1    | 1.2     | 0.01, 0, 0.08      | 0.003, 0.003, 0.0015      | Turret and shoulder motor |
| 2    | 1.0     | 0, 0, 0.11         | 0.0052, 0.0052, 0.00045   | Upper arm, elbow motor    |
| 3    | 0.8     | 0, 0, 0.10         | 0.0042, 0.0042, 0.00036   | Forearm, wrist motors     |
| 4    | 0.3     | 0, 0, −0.03        | 0.0002, 0.0002, 0.0001    | Wrist roll                |
| 5    | 0.25    | 0, 0, 0.02         | 0.0001, 0.0001, 0.00005   | Wrist pitch               |
| 6    | 0.1     | 0, 0, 0.05         | 0.00003, 0.00003, 0.00002 | Flange                    |

Products of inertia are zero. From them the model gives, in joint space and newton-metres:

- **Gravity torque** at a set of joint positions: what each joint holds at rest. Gravity points down in the world, so
  the base mount's orientation counts.
- **Inverse dynamics**: the joint torques of joint positions, speeds and accelerations, gravity included.
- **Mass matrix**: 6 × 6, symmetric and positive definite.

The arm host publishes the gravity torque at the measured joints in each snapshot, and the console prints it
(`gravity`).

### Part E — Measured cost

The arm model test times each call over 20 000 calls at varying joints and prints a `[METRIC]` line. The
measurement uses a Debug build, float, and one PC core in the development container:

| Call               | Time per call | Share of the 10 ms period |
|--------------------|---------------|---------------------------|
| Tool pose          | 0.4 µs        | 0.004 %                   |
| Jacobian           | 1.1 µs        | 0.01 %                    |
| Singularities      | 0.1 µs        | 0.001 %                   |
| Inverse kinematics | 0.9 µs        | 0.01 %                    |
| Gravity torque     | 2.3 µs        | 0.02 %                    |
| Inverse dynamics   | 2.4 µs        | 0.02 %                    |
| Mass matrix        | 4.0 µs        | 0.04 %                    |

The test fails if any call exceeds 2 ms, a fifth of the period.

### Part G — The driven arm: the model with its drives' rotors

The arm controllers and the collision observer need the inertia the drives move. On a geared joint the motor's
rotor is a large part of it. Its inertia times the gear ratio squared is of the order of the links' own on a 50:1
gear. The driven arm holds the arm model and each joint's reflected rotor inertia. It puts the rotor on the joint's
link as its armature. The links' dynamics, robotics-toolbox's chain dynamics model over those links, then carry the
rotors without a correction around them:

- **Mass matrix**: M(q) + diag(Jᵣ), in the dynamics' coordinates or in joint space. The joints' signs flip only the
  coupling terms, so each rotor stays on its own diagonal entry.
- **Inverse dynamics**: the arm's torques plus Jᵣ·q̈.
- **Gravity and Coriolis terms**: the arm's; a rotor turning about its own axis adds neither.

The arm builds one driven arm from the profile's motors and gear ratios and hands it to every arm controller and to the
collision observer. The model itself stays without rotors: the gravity torque, the arm controllers' reference
feed-forward and the tools' snapshots are the links' alone.

---

## Interfaces

### Provided

| Interface          | Purpose                           | Contract                                                            |
|--------------------|-----------------------------------|---------------------------------------------------------------------|
| Tool pose          | Joint positions to tool pose      | Any joint positions                                                 |
| Inverse kinematics | Tool pose to joint positions      | Within the limits and closest to the current joints, or the failure |
| Jacobian           | Joint velocities to tool twist    | 6 × 6, base frame, at the tool centre point                         |
| Singularities      | Which singularity the arm is near | Wrist, elbow, shoulder, independently                               |
| Link frames        | Joint positions to link poses     | Six poses, base frame, at each joint                                |
| Gravity torque     | Joint positions to holding torque | N·m per joint, joint space                                          |
| Inverse dynamics   | Motion to joint torques           | Positions, speeds, accelerations; gravity included                  |
| Mass matrix        | Joint positions to inertia        | 6 × 6, joint space, symmetric positive definite                     |
| Driven arm         | The model with the drives' rotors | Mass matrix and link dynamics with each rotor as armature (Part G)  |

### Required

| Interface         | Purpose                           | Contract                                            |
|-------------------|-----------------------------------|-----------------------------------------------------|
| robotics-toolbox  | OPW forward and inverse, Jacobian | Analytical OPW inverse kinematics, geometric Jacobian, float |
| robotics-toolbox  | Dynamics                          | Chain dynamics model (RNEA, CRBA), float            |
| Arm configuration | Geometry, mass, joint limits      | From the profile ([arm-profile.md](arm-profile.md)) |
| Joint motion      | Moves the arm, Cartesian moves included (phase 4) | The model only converts                  |
| Arm controllers   | Gravity compensation, torque feed-forward (phase 5) | They feed the torques to the joints    |
| Joint drives      | Friction and rotor inertia                        | Each joint's drive ([arm-dynamics.md](../theory/arm-dynamics.md)); the driven arm adds the rotors (Part G) |

---

## Data Model

| Entity   | Field          | Type / Unit                    | Range          | Notes                       |
|----------|----------------|--------------------------------|----------------|-----------------------------|
| Geometry | lengths        | 7 × m                          | c2, c3 above 0 | The OPW lengths             |
| Geometry | signs, offsets | 6 × ±1, 6 × rad                | —              | Joint space to model angles |
| Geometry | tool, base     | position m, roll pitch yaw rad | —              | Identity by default         |
| Mass     | links          | 6 × (kg, m, kg·m²)             | Part F         | Mass, centre, inertia       |
| Mass     | payload        | kg, m, kg·m²                   | Part F         | Relative to the flange      |
| Snapshot | tool pose      | position m, roll pitch yaw rad | —              | From the measured joints    |
| Snapshot | singularities  | wrist, elbow, shoulder         | —              | From the measured joints    |
| Snapshot | gravity torque | 6 × N·m                        | —              | From the measured joints    |

---

## Sequence Diagrams

```mermaid
sequenceDiagram
    participant C as Console
    participant H as Arm host
    participant M as Arm model
    H->>M: tool pose (measured joints), each control period
    M-->>H: pose, singularities, gravity torque
    H-->>C: snapshot
    C->>M: ik (target pose, current joints)
    M-->>C: joints, or unreachable / outside joint limits
```

---

## Constraints & Limitations

| Constraint | Value / Description                                                 |
|------------|---------------------------------------------------------------------|
| Memory     | No heap; the model holds the geometry and the limits                |
| Time       | Each call well under a fifth of the 10 ms period on the PC (Part E) |
| Shape      | OPW family only (see the theory's limitations)                      |

---

## Decisions

| # | Decision                    | Options                                     | Status                         |
|---|-----------------------------|---------------------------------------------|--------------------------------|
| 1 | Geometry description        | OPW, DH, both                               | decided: OPW                   |
| 2 | The real arm's dimensions   | Measured, from the CAD                      | open: the reference arm stands |
| 3 | Arms outside the OPW family | DH or product of exponentials, numerical IK | open: when such an arm appears |
| 4 | The real arm's masses       | Weighed, from the CAD                       | open: the reference arm stands |
