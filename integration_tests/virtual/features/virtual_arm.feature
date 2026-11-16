@virtual
Feature: The arm on six e-foc virtual motors
  The arm runs on six e-foc virtual motors, one per joint, each through a 50:1 gear and each carrying the load the
  arm's links put on its shaft. The motors run the e-foc firmware against a motor model in real time, and each drive supervises its following
  error: a shaft that lags its setpoint by more than 3 rad for 0.1 s faults.

  Background:
    Given the arm on six e-foc virtual motors
    When the arm is brought up and enabled

  Scenario: The arm lifts its shoulder against gravity and tracks the move
    When the arm moves the joints to 0.3, -0.5, 0.4, 0.2, 0.3, 0.1 rad
    Then the move shall be done within 8000 ms
    And within 2000 ms every joint shall be within 0.01 rad of 0.3, -0.5, 0.4, 0.2, 0.3, 0.1
    And the largest tracking error during the move shall stay below 0.1 rad

  @REQ-ARM-004
  Scenario: A collision that holds a joint back trips the arm on the drive's following error
    When the arm moves the joints to 0.3, -0.5, 0.4, 0.2, 0.3, 0.1 rad
    And a collision pushes joint 2 with 30 N·m
    Then within 1000 ms the arm shall trip with joint 2 faulted on a following error

  @REQ-ARM-005
  Scenario: A motor that stops answering trips the arm
    When motor 4 stops answering
    Then within 1500 ms the arm shall trip with joint 4 silent
