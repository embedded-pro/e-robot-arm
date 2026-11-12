#ifndef CORE_EXAMPLE_COMPONENT_IMPLEMENTATIONS_ACCUMULATOR_IMPL_HPP
#define CORE_EXAMPLE_COMPONENT_IMPLEMENTATIONS_ACCUMULATOR_IMPL_HPP

#include "core/example_component/interfaces/Accumulator.hpp"

namespace example
{
    class AccumulatorImpl final
        : public Accumulator
    {
    public:
        void Add(int32_t value) override;
        int32_t Total() const override;
        void Reset() override;

    private:
        int32_t total{ 0 };
    };
}

#endif
