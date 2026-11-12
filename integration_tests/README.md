# integration_tests

BDD integration tests using [cucumber-cpp](https://github.com/philips-software/amp-cucumber-cpp-runner)
(Gherkin features + C++ step definitions). Built only when `SCAFFOLD_BUILD_TESTS`
is on (the `host` preset enables it) and run via CTest.

```
integration_tests/
├── features/        # Gherkin .feature files (the specification)
│   └── accumulator.feature
├── support/         # Fixtures shared across steps (the system under test)
├── hooks/           # Before/After scenario hooks
├── steps/           # C++ step definitions binding Gherkin to the fixture
└── main/            # Test executable (cucumber_cpp.runner provides main)
```

Run them:

```bash
cmake --preset host
cmake --build --preset host-Debug
ctest --preset host -R embedded_scaffold.integration_tests
```

## How it fits together

- A **fixture** in `support/` holds the system under test and is stored in the
  cucumber `context` (`context.Emplace<T>()` / `context.Get<T>()`).
- **Steps** in `steps/` use `GIVEN` / `WHEN` / `THEN` / `STEP` macros whose
  regex matches lines in the `.feature` files.
- The **feature** files are the human-readable spec; keep them in sync with the
  product requirements.

## Add a scenario

1. Add a `Scenario:` to a `.feature` file (or add a new feature).
2. Implement any new `Given/When/Then` lines as steps in `steps/`.
3. Extend the fixture in `support/` if the steps need new hooks into the SUT.
