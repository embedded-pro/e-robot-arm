Feature: Motion generators
  Joint motion moves the six joints point to point (trapezoidal or S-curve), through joint waypoints, along a
  straight tool line, and by jogging, on six simulated joints under the arm's gravity-pd. Every way of moving keeps
  the reference within the joint limits, so the safety supervisor never stops the arm for it. At the pose the tool
  lines start from, gravity-pd leaves joints 1, 4 and 6 in a lightly damped swing of a few milliradians, which sets
  the tolerances on where the tool is.

  Background:
    Given an arm with six simulated joints, jerk limits and slow tool limits
    And the control loop is running
    And the joints are started in torque mode

  Scenario: An S-curve move arrives at its target
    Given the arm moves with the S-curve profile
    When the arm moves the joints to 0.5, -0.4, 0.3, -0.2, 0.1, 0.6 rad
    Then the move shall be done within 2500 ms
    And every joint shall be at its target

  Scenario: A new target replaces a running move, which is done once at the new target
    When the arm moves the joints to 0.5, 0, 0, 0, 0, 0 rad
    And after 300 ms the arm moves the joints to -0.3, 0.2, 0, 0, 0, 0 rad
    Then the move shall be done within 3000 ms
    And every joint shall be at its target

  @REQ-MOT-001
  Scenario: A path passes through every waypoint and rests at the last
    Given a waypoint at 0.4, 0.2, 0, 0, 0, 0 rad
    And a waypoint at -0.2, 0.4, 0.2, 0, 0, 0 rad
    And a waypoint at 0, 0, 0.3, 0, 0, 0 rad
    When the arm moves through the waypoints
    Then the path shall be done within 6000 ms, passing within 0.05 rad of every waypoint
    And every joint shall be at its target

  @REQ-MOT-002
  Scenario: The tool moves in a straight line
    When the arm moves the joints to 0.1, 0.5, 0.6, 0.2, 1.0, 0 rad
    Then the move shall be done within 3000 ms
    And every joint shall be at its target
    When the arm moves the tool in a straight line by 0, 0.03, -0.01 m
    Then the line shall be done within 3000 ms with the tool within 0.025 m of the line and at rest within 0.015 m of its target

  @REQ-MOT-002
  Scenario: A line the arm cannot follow moves nothing
    When the arm moves the joints to 0.1, 0.5, 0.6, 0.2, 1.0, 0 rad
    Then the move shall be done within 3000 ms
    And every joint shall be at its target
    When the arm moves the tool in a straight line by 2, 0, 0 m
    Then the line shall be refused, no move shall start and every joint shall hold within 0.02 rad

  @REQ-MOT-003
  Scenario: A jog comes to rest when the operator stops commanding it
    When the arm jogs joint 2 at 0.5 rad/s for 300 ms
    Then within 1000 ms the jog shall be done with every joint at rest

  @REQ-MOT-002
  Scenario: Teach two points, then move between them in a straight line
    When the arm moves the joints to 0.1, 0.5, 0.6, 0.2, 1.0, 0 rad
    Then the move shall be done within 3000 ms
    And every joint shall be at its target
    When the arm teaches the point "a"
    And the arm jogs the tool along the base y axis at 0.03 m/s for 1000 ms
    Then within 1000 ms the jog shall be done with every joint at rest
    When the arm teaches the point "b"
    And the arm goes to the point "a"
    Then the move shall be done within 3000 ms
    And every joint shall be at its target
    When the arm moves the tool in a straight line to the point "b"
    Then the line shall be done within 3000 ms with the tool within 0.025 m of the line and at rest within 0.015 m of its target
