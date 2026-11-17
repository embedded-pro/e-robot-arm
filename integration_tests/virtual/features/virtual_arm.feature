@virtual
Feature: The arm on six e-foc virtual motors
  The arm runs on six e-foc virtual motors, one per joint, each through a 50:1 gear and each carrying the load the
  arm's links put on its shaft. The motors run the e-foc firmware against a motor model in real time, in torque mode:
  the arm closes position with gravity-pd on the host, and supervises each joint's tracking error, so a joint that
  lags its reference by more than 0.1 rad for 0.1 s trips the arm. A momentum observer estimates the external torque on
  each joint, and one above the joint's collision limit, 10 N·m on the first three joints and 2 N·m on the wrist,
  trips the arm. gravity-pd and computed-torque have no integral action, so the joints settle short of the target by
  the drives' Coulomb friction over their stiffness; gravity-pid integrates it away.

  Background:
    Given the arm on six e-foc virtual motors
    When the arm is brought up and enabled

  Scenario: The arm lifts its shoulder against gravity and tracks the move
    When the arm moves the joints to 0.3, -0.5, 0.4, 0.2, 0.3, 0.1 rad
    Then the move shall be done within 8000 ms
    And within 2000 ms every joint shall be within 0.06 rad of 0.3, -0.5, 0.4, 0.2, 0.3, 0.1
    And the largest tracking error during the move shall stay below 0.1 rad
    And the largest external torque estimated during the move shall stay below 7 N·m

  Scenario: Computed torque tracks the move with the reference speed and acceleration
    When the arm controls its joints with "computed-torque"
    And the arm moves the joints to 0.3, -0.5, 0.4, 0.2, 0.3, 0.1 rad
    Then the move shall be done within 8000 ms
    And within 2000 ms every joint shall be within 0.06 rad of 0.3, -0.5, 0.4, 0.2, 0.3, 0.1
    And the largest tracking error during the move shall stay below 0.06 rad

  Scenario: A PID with gravity feed-forward settles every joint on the target despite the drives' friction
    When the arm controls its joints with "gravity-pid"
    And the arm moves the joints to 0.3, -0.5, 0.4, 0.2, 0.3, 0.1 rad
    Then the move shall be done within 8000 ms
    And within 2000 ms every joint shall be within 0.015 rad of 0.3, -0.5, 0.4, 0.2, 0.3, 0.1
    And the largest tracking error during the move shall stay below 0.04 rad

  @REQ-ARM-016
  Scenario: A collision trips the arm on the external torque its momentum observer estimates
    When the arm moves the joints to 0.3, -0.5, 0.4, 0.2, 0.3, 0.1 rad
    And a collision pushes joint 2 with 30 N·m
    Then within 300 ms the arm shall trip with joint 2 on a collision

  @REQ-ARM-005
  Scenario: A motor that stops answering trips the arm
    When motor 4 stops answering
    Then within 1500 ms the arm shall trip with joint 4 silent
