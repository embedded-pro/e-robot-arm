Feature: Arm controllers switched at run time
  The arm offers more than one arm controller, and one of them at a time turns the motion's reference into the
  joints' setpoints: pass-through, which sends the reference as it is, and smoothed, which low-passes it. The
  operator switches between them while the arm runs, without the setpoints stepping.

  Background:
    Given an arm with six simulated joints on one CAN bus
    And the control loop is running
    When the arm is brought up
    And the arm is enabled

  @REQ-ARM-013
  Scenario: The smoothed controller takes over a running move without a setpoint step
    Then the active arm controller shall be "pass-through"
    When the arm moves the joints to 0.5, -0.4, 0.3, -0.2, 0.1, 0.6 rad
    And 300 ms pass
    And the arm selects the "smoothed" arm controller
    Then the active arm controller shall be "smoothed"
    And for 500 ms no joint shall move more than 0.015 rad in one control period
    And the move shall be done within 1000 ms
    And within 500 ms every joint shall be at its target

  @REQ-ARM-013
  Scenario: A switch that would step the setpoints is refused while a move runs
    When the arm selects the "smoothed" arm controller
    And the arm moves the joints to 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 rad
    And 200 ms pass
    Then selecting the "pass-through" arm controller shall be refused
    And the active arm controller shall be "smoothed"

  @REQ-ARM-013
  Scenario: Disabling the active arm controller stops the arm where it is
    When the arm moves the joints to 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 rad
    And 300 ms pass
    And the arm disables the "pass-through" arm controller
    Then no arm controller shall be active
    And the move shall be aborted and the joints shall hold their positions for 500 ms
