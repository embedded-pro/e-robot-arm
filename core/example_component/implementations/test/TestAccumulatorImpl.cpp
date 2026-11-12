#include "core/example_component/implementations/AccumulatorImpl.hpp"
#include "gmock/gmock.h"

namespace
{
    class AccumulatorImplTest
        : public testing::Test
    {
    protected:
        example::AccumulatorImpl accumulator;
    };
}

TEST_F(AccumulatorImplTest, starts_at_zero)
{
    EXPECT_EQ(0, accumulator.Total());
}

TEST_F(AccumulatorImplTest, accumulates_added_values)
{
    accumulator.Add(2);
    accumulator.Add(3);

    EXPECT_EQ(5, accumulator.Total());
}

TEST_F(AccumulatorImplTest, reset_clears_the_total)
{
    accumulator.Add(7);
    accumulator.Reset();

    EXPECT_EQ(0, accumulator.Total());
}
