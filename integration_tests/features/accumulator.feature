Feature: Accumulator
  The example accumulator sums integer values. This feature exists to demonstrate
  the BDD integration-test harness (cucumber-cpp + Gherkin) wired into the scaffold.

  Scenario: Adding two values produces their sum
    Given a fresh accumulator
    When I add 2 and 3
    Then the total shall be 5

  Scenario: Resetting clears the running total
    Given a fresh accumulator
    When I add 2 and 3
    And I reset the accumulator
    Then the total shall be 0
