#include "core/example_component/implementations/AccumulatorImpl.hpp"
#include <cstdio>

// Minimal application entry point.
//
// A real target constructs its platform Board (see platform_implementations/) and
// injects hardware peripherals into the application logic via the constructor —
// never via global state. Here we keep it to the bare shape of a `main`.
int main()
{
    example::AccumulatorImpl accumulator;
    accumulator.Add(2);
    accumulator.Add(3);

    std::printf("accumulator total = %d\n", accumulator.Total());

    return 0;
}
