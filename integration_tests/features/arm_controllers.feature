Feature: Arm controllers switched at run time
  The arm turns the motion's reference into the joints' torques through its active arm controller: gravity-pd, a PD
  on the position error with the gravity torque, or computed-torque, the arm's inverse dynamics of the reference
  acceleration. The operator switches between them while the arm runs. In torque mode nothing else holds the joints,
  so the controller that drives them cannot be disabled until the arm is disabled.

  Background:
    Given an arm with six simulated joints on one CAN bus
    And the control loop is running
    When the arm is brought up
    And the arm is enabled

  @REQ-ARM-013
  Scenario: Computed torque takes over a running move and brings the joints onto the target
    Then the active arm controller shall be "gravity-pd"
    When the arm moves the joints to 0.5, -0.4, 0.3, -0.2, 0.1, 0.6 rad
    And 300 ms pass
    And the arm selects the "computed-torque" arm controller
    Then the active arm controller shall be "computed-torque"
    And the move shall be done within 1500 ms
    And every joint shall be at its target

  @REQ-ARM-013
  Scenario: The controller that drives the joints is disabled only once the arm is disabled
    Then the active arm controller shall be "gravity-pd"
    When the arm moves the joints to 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 rad
    And 300 ms pass
    Then disabling the "gravity-pd" arm controller shall be refused
    And the move shall be done within 2000 ms
    When the arm is disabled
    And the arm disables the "gravity-pd" arm controller
    Then no arm controller shall be active
