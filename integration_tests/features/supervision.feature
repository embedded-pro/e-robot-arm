Feature: Supervision of the moving arm
  While the arm drives its joints, the safety supervisor checks every control period the torque it sends, how fast
  each joint turns and the reference the arm controller follows. A torque above a joint's limit is clamped and
  reported; a joint too fast or a reference beyond the motion limits stops the arm protectively: the joints stay
  powered and hold the pose, and a re-arm drives them again from there.

  @REQ-SAF-005 @REQ-SAF-001 @REQ-SAF-003
  Scenario: A joint faster than its speed limit stops the arm, which holds its pose and moves again once re-armed
    Given an arm with six simulated joints whose joint 1 may turn at most 0.5 radians per second
    And the control loop is running
    When the arm is brought up
    And the arm is enabled
    And the arm moves the joints to 1, 0, 0, 0, 0, 0 rad
    Then within 1500 ms the arm shall stop protectively, naming joint 1 on its "speed"
    And for 1000 ms every joint shall stay running within 0.02 rad of where the arm stopped
    When the arm is re-armed
    Then the arm shall be enabled with every joint running in torque mode
    When the arm moves joint 1 0.1 rad further
    Then the move shall be done within 1500 ms

  @REQ-SAF-004
  Scenario: A torque above a joint's limit is clamped to it and reported
    Given an arm with six simulated joints whose joint 2 may drive at most 2 N·m
    And the control loop is running
    When the arm is brought up
    And the arm is enabled
    And the arm moves the joints to 0, 0.8, 0, 0, 0, 0 rad
    Then for 1500 ms joint 2 shall drive at most 2 N·m, and the arm shall report its torque clamped

  @REQ-SAF-006
  Scenario: A reference faster than the motion limits allow stops the arm protectively
    Given an arm with six simulated joints on one CAN bus
    And the control loop is running
    When the arm is brought up
    And the arm is enabled
    And the arm is handed a reference that drives joint 3 at 3 radians per second
    Then within 100 ms the arm shall stop protectively, naming joint 3 on its "reference speed"

  @REQ-ARM-015
  Scenario: A joint pushed past its position limit stops the arm, which reports the joint and its position and holds
    Given an arm with six simulated joints whose joint 1 may move from -0.05 to 0.05 rad
    And the control loop is running
    When the arm is brought up
    And the arm is enabled
    And joint 1 is pushed with 0.3 N·m
    Then within 1000 ms the arm shall stop protectively, naming joint 1 on its "position limit"
    And the arm shall report joint 1 measured beyond 0.05 rad
    When the push on joint 1 ends
    Then for 1000 ms every joint shall stay running within 0.02 rad of where the arm stopped
