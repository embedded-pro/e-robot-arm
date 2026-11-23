Feature: The arm profile
  The arm takes its configuration from one profile file: its control timing and, per joint, the node, gear ratio,
  direction, zero offset, limits and motor controller. A profile with an unknown, missing or out-of-range item is
  refused, naming it, and a profile written from a configuration reads back to the same configuration.

  @REQ-ARM-010
  Scenario: An arm built from its profile keeps the profile's limits, and the profile reads back word for word
    Given an arm profile whose joint 1 may move from -0.5 to 0.5 rad
    When the profile is read
    Then the profile shall be accepted and written back word for word
    When an arm with six simulated joints is built from the profile read
    And the control loop is running
    And the arm is brought up
    And the arm is enabled
    And the arm moves the joints to 0.4, 0, 0, 0, 0, 0 rad
    Then the move shall be done within 3000 ms
    When the arm moves the joints to 0.6, 0, 0, 0, 0, 0 rad
    Then the move shall be refused and no joint shall move

  @REQ-ARM-010
  Scenario: A profile with an out-of-range item is refused, naming the item
    Given an arm profile whose joint 1 may move from -0.5 to 0.5 rad
    And the profile's joint 2 gear ratio is changed to 0
    When the profile is read
    Then the profile shall be refused, naming "joints[1].transmission.ratio"
