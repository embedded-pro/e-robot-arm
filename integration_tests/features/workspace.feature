Feature: Workspace and self-collision
  The arm checks every move against its workspace, its forbidden zones and itself before it starts it, over the
  arm model with a capsule on each link: a move whose target or path fails is refused and no joint moves.

  Background:
    Given an arm with six simulated joints whose workspace forbids the box from 0.15, -0.2, 0.3 to 0.6, 0.2, 0.7 m
    And the control loop is running
    And the joints are started in torque mode

  @REQ-SAF-007
  Scenario: A move whose path enters a forbidden zone moves nothing
    When the arm moves the joints to 0, 1.2, 0, 0, 0, 0 rad
    Then the move shall be refused and no joint shall move

  Scenario: A move that stays clear of the zone is done
    When the arm moves the joints to 0, -0.5, 0.3, 0, 0, 0 rad
    Then the move shall be done within 2000 ms

  @REQ-SAF-008
  Scenario: A move that folds the forearm onto the arm itself moves nothing
    When the arm moves the joints to 0, 0, 3, 0, 0, 0 rad
    Then the move shall be refused and no joint shall move
