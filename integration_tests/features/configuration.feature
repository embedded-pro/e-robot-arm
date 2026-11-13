Feature: Joint motor-controller configuration
  Each joint's e-foc motor controller (the algorithm of each loop and its gains) is applied over CAN while the arm
  is ready, and read back to confirm it.

  Background:
    Given an arm with six simulated joints on one CAN bus
    And the control loop is running

  @REQ-ARM-009
  Scenario: A joint's controller is applied and read back while the arm is ready
    When the arm is brought up
    Then the arm shall be ready
    When joint 2 is given sliding-mode current, LQI speed and two-DOF position control with a speed bandwidth of 150 radians per second
    Then the configuration shall be applied to joint 2
    And reading joint 2 back shall report exactly what was applied

  @REQ-ARM-009
  Scenario: A running arm refuses a configuration
    When the arm is brought up
    And the arm is enabled
    When joint 2 is given sliding-mode current, LQI speed and two-DOF position control with a speed bandwidth of 150 radians per second
    Then the configuration shall be refused
