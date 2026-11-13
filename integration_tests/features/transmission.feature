Feature: Joint transmissions
  Each joint drives its motor through a gear ratio, a direction and a zero offset. The arm commands and reports
  joints in joint space and checks its limits there; only the joint bus sees motor angles.

  Background:
    Given an arm whose joint 3 turns its motor 10 times, reversed, with its zero at 0.5 rad
    And the control loop is running

  @REQ-ARM-007
  Scenario: A geared joint moves its motor through the transmission
    Given the joints are started in position mode
    When the arm moves the joints to 0.2, 0.2, 0.9, 0.2, 0.2, 0.2 rad
    Then the move shall be done within 2000 ms
    And the arm shall report every joint at its target
    And motor 3 shall be at -4 rad
