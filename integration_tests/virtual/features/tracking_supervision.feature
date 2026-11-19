@virtual
Feature: Tracking supervision on the virtual arm
  With every joint's collision and speed limits raised out of reach, a collision that holds a joint back is left to the arm's
  tracking-error supervision: a joint that lags its reference by more than 0.1 rad for 0.1 s stops the arm, which then
  holds its pose. The push on the shoulder drags the elbow too, and whichever lags long enough first stops it.

  Background:
    Given the arm on six e-foc virtual motors with every joint's collision limit at 1000 N·m and speed limit at 100 rad/s
    When the arm is brought up and enabled

  @REQ-ARM-014
  Scenario: A collision that holds a joint back stops the arm on its tracking error
    When the arm moves the joints to 0.3, -0.5, 0.4, 0.2, 0.3, 0.1 rad
    And a collision pushes joint 2 with 30 N·m
    Then within 1000 ms the arm shall stop protectively on a joint's tracking error
