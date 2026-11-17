@virtual
Feature: Tracking supervision on the virtual arm
  With every joint's collision limit raised out of reach, a collision that holds a joint back is left to the arm's
  tracking-error supervision: a joint that lags its reference by more than 0.1 rad for 0.1 s trips the arm.

  Background:
    Given the arm on six e-foc virtual motors with every joint's collision limit at 1000 N·m
    When the arm is brought up and enabled

  @REQ-ARM-014
  Scenario: A collision that holds a joint back trips the arm on its tracking error
    When the arm moves the joints to 0.3, -0.5, 0.4, 0.2, 0.3, 0.1 rad
    And a collision pushes joint 2 with 30 N·m
    Then within 1000 ms the arm shall trip with joint 2 on its tracking error
