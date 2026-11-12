#include "core/example_component/implementations/AccumulatorImpl.hpp"
#include <cstdio>
#include <cstdlib>

// Example host tool: sums its integer arguments using the shared core library.
//
//     embedded_scaffold.tool.example 2 3 4   ->  total = 9
//
// Tools live on the host and may use the heap / standard library freely; they
// reuse core libraries rather than duplicating logic.
int main(int argc, const char* const argv[])
{
    example::AccumulatorImpl accumulator;

    for (int i = 1; i != argc; ++i)
        accumulator.Add(static_cast<int32_t>(std::atoi(argv[i])));

    std::printf("total = %d\n", accumulator.Total());

    return 0;
}
