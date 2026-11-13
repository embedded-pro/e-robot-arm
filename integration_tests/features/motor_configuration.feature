Feature: Joint motor configuration
  A joint's motor model and drive limits come from a reference motor or custom values. While bringing itself up, the
  arm applies each motor its configuration names and reads it back before the joint's motor controller.

  Background:
    Given an arm whose configuration gives joint 3 the "Anaheim BLY172S-24V-4000" motor
    And the control loop is running

  @REQ-ARM-012
  Scenario: Bring-up applies and confirms a joint's motor before the arm is ready
    When the arm is brought up
    Then the arm shall be ready within 500 ms
    And joint 3 shall hold the "Anaheim BLY172S-24V-4000" motor
    And joint 1 shall hold the "Teknic M-2310P-LN-04K" motor

  @REQ-ARM-012
  Scenario: A motor that reads back different stops the bring-up and names the joint's motor
    Given joint 3 ignores changes to its resistance
    When the arm is brought up
    Then the arm shall be misconfigured within 500 ms, naming the motor of joint 3
    And the arm shall refuse to be enabled
