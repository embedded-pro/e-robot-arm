Feature: Every arm controller holds the arm and carries its moves
  The arm offers a catalogue of arm controllers, from a PD with gravity to impedance, operational-space and adaptive
  laws. Whichever one the operator selects, before enabling the arm or while it holds a pose, the arm keeps its
  joints where they are at the switch and brings them onto the targets it is moved to. cartesian-impedance holds the
  light wrist's orientation with a soft spring, so started from the stretched home it is still about 0.01 rad off a
  second after its first move.

  @REQ-CTL-001
  Scenario Outline: A controller takes over a held pose without moving it and then carries a move
    Given an arm with six simulated joints <allowing> experimental controllers
    And the control loop is running
    When the arm is brought up
    And the arm is enabled
    And the arm moves the joints to 0.2, 0.4, 0.5, 0.3, 0.8, -0.2 rad
    Then the move shall be done within 3000 ms
    And a second after the move every joint shall be within 0.005 rad of its target
    When the arm selects the "<controller>" arm controller
    Then the active arm controller shall be "<controller>"
    And for 1000 ms every joint shall stay within 0.005 rad of where it was
    When the arm moves the joints to 0.4, 0.2, 0.7, 0.0, 0.5, 0.1 rad
    Then the move shall be done within 3000 ms
    And a second after the move every joint shall be within <settled> rad of its target

    Examples:
      | controller          | allowing | settled |
      | gravity-pd          | without  | 0.005   |
      | computed-torque     | without  | 0.005   |
      | gravity-pid         | without  | 0.005   |
      | joint-impedance     | without  | 0.005   |
      | cartesian-impedance | without  | 0.005   |
      | operational-space   | without  | 0.005   |
      | slotine-li          | with     | 0.005   |

  @REQ-CTL-002
  Scenario Outline: A controller selected before the arm is enabled brings the joints onto a target
    Given an arm with six simulated joints <allowing> experimental controllers
    And the control loop is running
    When the arm selects the "<controller>" arm controller
    And the arm is brought up
    And the arm is enabled
    And the arm moves the joints to 0.3, -0.3, 0.4, 0.2, 0.6, 0.1 rad
    Then the move shall be done within 3000 ms
    And a second after the move every joint shall be within <settled> rad of its target

    Examples:
      | controller          | allowing | settled |
      | gravity-pd          | without  | 0.005   |
      | computed-torque     | without  | 0.005   |
      | gravity-pid         | without  | 0.005   |
      | joint-impedance     | without  | 0.005   |
      | cartesian-impedance | without  | 0.03    |
      | operational-space   | without  | 0.005   |
      | slotine-li          | with     | 0.005   |
