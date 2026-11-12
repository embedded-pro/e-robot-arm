#ifndef CORE_EXAMPLE_COMPONENT_INTERFACES_ACCUMULATOR_HPP
#define CORE_EXAMPLE_COMPONENT_INTERFACES_ACCUMULATOR_HPP

#include <cstdint>

namespace example
{
    // Example abstract interface.
    //
    // Note the deliberately non-virtual, protected destructor: the project forbids
    // pure virtual destructors (`virtual ~T() = 0`) because they add flash/RAM
    // overhead on embedded targets. Ownership stays with the concrete type.
    class Accumulator
    {
    public:
        Accumulator() = default;
        Accumulator(const Accumulator& other) = delete;
        Accumulator& operator=(const Accumulator& other) = delete;

        virtual void Add(int32_t value) = 0;
        virtual int32_t Total() const = 0;
        virtual void Reset() = 0;

    protected:
        ~Accumulator() = default;
    };
}

#endif
