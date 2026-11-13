Feature: Arm start-up sequence
  The arm is brought up, enabled and moved in a fixed order: contracts, calibration, control mode and start. A
  faulted joint trips it, and it moves again only after it has been re-armed and enabled once more.

  Background:
    Given an arm with six simulated joints on one CAN bus
    And the control loop is running

  @REQ-ARM-008
  Scenario: The arm moves after bring-up and enable, and again only after a re-arm
    When the arm is brought up
    Then the arm shall be ready
    And the arm shall refuse a move
    When the arm is enabled
    Then the arm shall be enabled with every joint running in position mode
    When the arm moves the joints to 0.3, 0.3, 0.3, 0.3, 0.3, 0.3 rad
    Then the move shall be done within 1000 ms
    When joint 3 faults
    Then the arm shall be tripped
    And the arm shall refuse a move and refuse to be enabled
    When the faults are cleared and the arm is re-armed
    Then the arm shall be ready
    When the arm is enabled
    And the arm moves the joints to -0.2, -0.2, -0.2, -0.2, -0.2, -0.2 rad
    Then the move shall be done within 1000 ms
    And every joint shall be at its target
