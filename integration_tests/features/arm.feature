Feature: Arm on six e-foc joints
  The arm controller drives six simulated e-foc motor nodes on one CAN bus, through the joint bus, the safety
  supervisor, joint motion and the control loop. Each simulated node runs e-foc-protocol's motor category server
  in torque mode, and the six joints move together with the arm model's dynamics under the arm's gravity-pd.

  Background:
    Given an arm with six simulated joints on one CAN bus
    And the control loop is running

  @REQ-ARM-001
  Scenario: The arm moves no joint until every contract is known to be compatible
    When the arm is commanded to start the joints
    Then the command shall be refused
    When the arm checks the contracts of its joints
    Then the contracts shall be reported compatible
    And the arm shall start every joint in torque mode

  @REQ-ARM-002
  Scenario: The six joints move together to a target
    Given the joints are started in torque mode
    When the arm moves the joints to 0.5, -0.4, 0.3, -0.2, 0.1, 0.6 rad
    Then halfway through the move every joint shall be between its start and its target
    And the move shall be done within 1000 ms
    And every joint shall be at its target

  @REQ-ARM-003
  Scenario: A target outside the joint limits moves nothing
    Given the joints are started in torque mode
    When the arm moves the joints to 0, 0, 4, 0, 0, 0 rad
    Then the move shall be refused and no joint shall move

  @REQ-ARM-004
  Scenario: A faulted joint stops every joint until the arm is re-armed
    Given the joints are started in torque mode
    When joint 3 faults
    Then every joint shall have been emergency stopped
    And the arm shall report joint 3 faulted
    And the arm shall refuse to move
    When the faults are cleared, the arm is re-armed and the joints are started again
    And the arm moves the joints to 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 rad
    Then the move shall be done within 1000 ms
    And every joint shall be at its target

  @REQ-ARM-005
  Scenario: A silent joint stops every joint
    Given the joints are started in torque mode
    When joint 5 stops answering
    Then the arm shall report joint 5 silent
    And every joint but joint 5 shall have been emergency stopped

  @REQ-ARM-006
  Scenario: A command issued during a telemetry burst reaches every joint
    Given the contracts are compatible and torque mode is selected
    When the arm requests telemetry and starts the joints in the same turn
    Then every joint shall be running within 20 ms
    And the start shall be reported delivered and no command shall have failed

  @REQ-SAF-010
  Scenario: The hardware stop stops every joint, and the arm re-arms only once it has closed
    Given the joints are started in torque mode
    When the hardware stop opens
    Then every joint shall have been emergency stopped
    And the arm shall refuse to move
    And the arm shall refuse to re-arm
    When the hardware stop closes
    Then the arm shall re-arm
