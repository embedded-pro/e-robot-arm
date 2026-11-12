---
description: "embedded-scaffold testing guidelines: prefer TEST_F for fixture-based tests, TYPED_TEST for multi-type tests, host simulation for integration, Arrange-Act-Assert pattern. StrictMock required; NiceMock forbidden."
applyTo: "**/test/**"
---

# embedded-scaffold Testing Guidelines

## File Structure

- Unit test files: `core/<component>/implementations/test/Test{ComponentName}.cpp`
- Integration tests (BDD): `integration_tests/`
- CMake: tests added via `add_subdirectory(test)` and registered with
  `emil_add_test` / `emil_build_for`

## Framework

- GoogleTest for assertions (`TEST_F`, `TYPED_TEST`, `TEST`)
- GoogleMock for mocking interfaces (`testing::StrictMock<>`)
- Heap allocation (`std::make_unique`, `std::vector`, etc.) is permitted in
  host-side unit tests; the no-heap rule applies only to embedded target code and
  ISR-reachable paths

## TDD — Red-Green-Refactor

1. **Red** — write a failing test that describes one specific behaviour. It must
   fail before any production code exists.
2. **Green** — write the minimum production code to make it pass.
3. **Refactor** — clean up code and test while keeping all tests green.
4. Repeat.

Tests are executable specifications. Write tests **before** implementation.

## Fixture Test Pattern (single type)

```cpp
#include "core/example_component/implementations/AccumulatorImpl.hpp"
#include <gtest/gtest.h>

namespace
{
    class AccumulatorImplTest : public ::testing::Test
    {
    protected:
        example::AccumulatorImpl accumulator;
    };
}

TEST_F(AccumulatorImplTest, accumulates_added_values)
{
    // Arrange
    // Act
    accumulator.Add(2);
    accumulator.Add(3);
    // Assert
    EXPECT_EQ(5, accumulator.Total());
}
```

## Typed Test Pattern (multiple numeric types — for templated algorithms)

```cpp
#include "numerical/filters/passive/FirFilter.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    class TestFirFilter : public ::testing::Test {};

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestFirFilter, TestTypes);
}

TYPED_TEST(TestFirFilter, produces_correct_output)
{
    // Arrange, Act, Assert
}
```

## Rules

- Prefer `TEST_F` when tests share fixture state or setup/teardown logic
- Use plain `TEST()` for simple, stateless tests without shared setup
- Fixture class inside anonymous `namespace {}`; test macros **outside** it
- Include `<gtest/gtest.h>` (use `<gmock/gmock.h>` only when gmock matchers are needed)
- Use `testing::StrictMock<MockType>` for ALL mock instances — `NiceMock` and
  `NaggyMock` are **FORBIDDEN**

## Test Quality

- Descriptive test names stating scenario and expected outcome
- One behaviour per test
- `EXPECT_NEAR` with explicit tolerance for floating-point assertions (not `EXPECT_EQ`)
- Tests must not require hardware — use host stubs from
  `targets/platform_implementations/host/`
- Arrange-Act-Assert pattern; Allman brace style applies to test code too
