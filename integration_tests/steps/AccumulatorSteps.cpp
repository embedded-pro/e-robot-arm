#include "cucumber_cpp/Steps.hpp"
#include "integration_tests/support/AccumulatorFixture.hpp"
#include <gtest/gtest.h>

using namespace integration;

GIVEN(R"(a fresh accumulator)")
{
    context.Emplace<AccumulatorFixture>();
}

WHEN(R"(I add 2 and 3)")
{
    auto& fixture = context.Get<AccumulatorFixture>();
    fixture.accumulator.Add(2);
    fixture.accumulator.Add(3);
}

STEP(R"(I reset the accumulator)")
{
    context.Get<AccumulatorFixture>().accumulator.Reset();
}

THEN(R"(the total shall be 5)")
{
    EXPECT_EQ(5, context.Get<AccumulatorFixture>().accumulator.Total());
}

THEN(R"(the total shall be 0)")
{
    EXPECT_EQ(0, context.Get<AccumulatorFixture>().accumulator.Total());
}
