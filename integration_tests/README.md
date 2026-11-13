# integration_tests

BDD integration tests using [cucumber-cpp](https://github.com/philips-software/amp-cucumber-cpp-runner)
(Gherkin features + C++ step definitions). Built only when `E_ROBOT_ARM_BUILD_TESTS`
is on (the `host` preset enables it) and run via CTest.

```
integration_tests/
├── features/        # Gherkin .feature files (the specification)
│   ├── arm.feature           #   the arm against six simulated e-foc joints (REQ-ARM-001…006)
│   ├── transmission.feature  #   joint transmissions (REQ-ARM-007)
│   └── lifecycle.feature     #   the start-up sequence (REQ-ARM-008)
├── support/         # Fixtures shared across steps (the system under test)
├── hooks/           # Before/After scenario hooks
├── steps/           # C++ step definitions binding Gherkin to the fixture
└── main/            # Test executable (cucumber_cpp.runner provides main)
```

Run them:

```bash
cmake --preset host
cmake --build --preset host-Debug
ctest --preset host -R e_robot_arm.integration_tests
```

## How it fits together

- A **fixture** in `support/` holds the system under test and is stored in the
  cucumber `context` (`context.Emplace<T>()` / `context.Get<T>()`).
- **Steps** in `steps/` use `GIVEN` / `WHEN` / `THEN` / `STEP` macros whose
  regex matches lines in the `.feature` files.
- The **feature** files are the human-readable spec; keep them in sync with the
  product requirements.

## The arm scenarios

`arm.feature` runs the real `arm::Arm` (joint bus, safety supervisor, joint motion, control loop) against six
`SimulatedJoint`s on one in-process `VirtualCanBus`, from `tools/simulated_arm/` (shared with the simulator and the
console). Each simulated joint is a node built on e-foc-protocol's `FocMotorCategoryServer`: it answers contract
queries, telemetry requests and commands as an e-foc node does. The scenarios give it no lag, so it follows its
position setpoints exactly. A scenario can make a joint fault or go silent. Time is the test clock
(`infra::ClockFixture`), so the scenarios are deterministic and run in milliseconds. Each scenario is tagged with
the requirement it verifies (`documentation/requirements/arm/arm.yaml`).

## Add a scenario

1. Add a `Scenario:` to a `.feature` file (or add a new feature).
2. Implement any new `Given/When/Then` lines as steps in `steps/`.
3. Extend the fixture in `support/` if the steps need new hooks into the SUT.
