#include "core/example_component/implementations/AccumulatorImpl.hpp"

namespace example
{
    void AccumulatorImpl::Add(int32_t value)
    {
        total += value;
    }

    int32_t AccumulatorImpl::Total() const
    {
        return total;
    }

    void AccumulatorImpl::Reset()
    {
        total = 0;
    }
}
