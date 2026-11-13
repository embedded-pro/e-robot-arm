Feature: Motor controllers applied at bring-up
  The arm's configuration names a motor controller for some joints. While bringing itself up, the arm applies each
  of them and reads it back before it can be enabled.

  Background:
    Given an arm whose configuration gives joints 2 and 5 LQR position control with a speed bandwidth of 120 radians per second
    And the control loop is running

  @REQ-ARM-011
  Scenario: Bring-up applies and confirms each configured motor controller before the arm is ready
    When the arm is brought up
    Then the arm shall be ready within 500 ms
    And joints 2 and 5 shall run LQR position control with a speed bandwidth of 120 radians per second
    And joint 1 shall keep its default motor controller

  @REQ-ARM-011
  Scenario: A motor controller that reads back different stops the bring-up and names the joint
    Given joint 5 ignores changes to its speed bandwidth
    When the arm is brought up
    Then the arm shall be misconfigured within 500 ms, naming joint 5
    And the arm shall refuse to be enabled
