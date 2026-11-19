@virtual
Feature: Collision supervision on the virtual arm
  With every joint's speed limit raised out of reach and its collision limit at 10 N·m, a push on the shoulder near the
  drive's peak torque through the gear is left to the momentum observer: the estimated external torque passes the
  joint's limit and the arm stops protectively, holding its pose. The push goes on, the hold cannot resist it, and the
  arm trips on the lost hold. With the speed rule in reach, the shoulder, which the elbow no longer outweighs, can pass
  its speed limit first and trip the arm on that instead.

  Background:
    Given the arm on six e-foc virtual motors with every joint's collision limit at 10 N·m and speed limit at 100 rad/s
    When the arm is brought up and enabled

  @REQ-ARM-016 @REQ-SAF-002
  Scenario: A collision stops the arm protectively, and a push the hold cannot resist ends in an emergency stop
    When the arm moves the joints to 0.3, -0.5, 0.4, 0.2, 0.3, 0.1 rad
    And a collision pushes joint 2 with 30 N·m
    Then within 300 ms the arm shall stop protectively with joint 2 on a collision
    And within 2000 ms the arm shall trip on a lost hold
