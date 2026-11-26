Feature: Without a tracking controller the simulated arm moves with its own dynamics
  On a simulation the arm offers two passive controllers. free sends no torque, so gravity moves the arm.
  gravity-float cancels gravity, so the arm stays where it is left. Neither takes a move. A tracking controller
  selected afterwards catches the arm where it is. The reference arm's joints are direct drive, so a free fall soon
  passes the speed rule and stops the arm: the catch comes first.

  @REQ-CTL-003
  Scenario: A floating arm stays where it is left
    Given an arm with six simulated joints on a simulation
    And the control loop is running
    When the arm is brought up
    And the arm is enabled
    And the arm moves the joints to 0.2, 0.4, 0.5, 0.3, 0.8, -0.2 rad
    Then the move shall be done within 3000 ms
    When 1000 ms pass
    And the arm selects the "gravity-float" arm controller
    Then the active arm controller shall be "gravity-float"
    And for 1000 ms every joint shall stay within 0.01 rad of where it was
    And the arm shall refuse a move

  @REQ-CTL-004
  Scenario: A free arm falls under gravity and a tracking controller catches it
    Given an arm with six simulated joints on a simulation
    And the control loop is running
    When the arm is brought up
    And the arm is enabled
    And the arm moves the joints to 0.2, 0.4, 0.5, 0.3, 0.8, -0.2 rad
    Then the move shall be done within 3000 ms
    When 1000 ms pass
    And the arm selects the "free" arm controller
    Then within 100 ms joint 3 shall move more than 0.05 rad
    And the arm shall refuse a move
    When the arm selects the "gravity-pd" arm controller
    And 1000 ms pass
    Then the arm shall still be enabled
    And for 1000 ms every joint shall stay within 0.005 rad of where it was
