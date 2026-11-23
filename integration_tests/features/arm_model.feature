Feature: The arm model
  The arm knows its geometry and the mass of its links. From the measured joints it gives the tool pose and the
  singularity the arm is near; from a tool pose it gives the joints that reach it, within the joint limits and
  closest to where the arm is, or says why there are none; and it gives the torque each joint holds against gravity.

  @REQ-MOD-001
  Scenario: The tool pose follows the measured joints and names the singularity the arm is near
    Given an arm with six simulated joints on one CAN bus
    And the control loop is running
    When the arm is brought up
    And the arm is enabled
    Then the tool pose of the measured joints shall be x 0.020, y 0, z 0.710 m
    And the arm shall name the wrist and elbow singularities and not the shoulder
    When the arm moves the joints to 0, 0.5, -1, 0, 0.5, 0 rad
    Then the move shall be done within 3000 ms
    And the tool pose of the measured joints shall be x 0.020, y 0, z 0.649 m
    And the arm shall name no singularity

  @REQ-MOD-002
  Scenario: The joints for a pose are the ones closest to the arm, and a pose out of reach is refused saying why
    Given an arm with six simulated joints whose joint 1 may move from -0.1 to 0.1 rad
    And the control loop is running
    When the arm is brought up
    And the arm is enabled
    And the arm moves the joints to 0, 0.5, -1, 0, 0.5, 0 rad
    Then the move shall be done within 3000 ms
    When the arm is asked for the joints that put the tool where it is now
    Then the answer shall be the measured joints
    When the arm is asked for the joints that put the tool at x 2, y 0, z 0 m, roll 0, pitch 0, yaw 0 rad
    Then the arm shall refuse the pose as "unreachable"
    When the arm is asked for the joints that put the tool where joints 0.5, 0.5, -1, 0, 0.5, 0 rad would
    Then the arm shall refuse the pose as "outside the joint limits"

  @REQ-MOD-003
  Scenario: A joint held away from upright drives the gravity torque the arm reports
    Given an arm with six simulated joints on one CAN bus
    And the control loop is running
    When the arm is brought up
    And the arm is enabled
    And the arm moves the joints to 0, 0.5, -1, 0, 0.5, 0 rad
    Then the move shall be done within 3000 ms
    And for 500 ms joint 2 shall hold with the gravity torque the arm reports for the measured joints
