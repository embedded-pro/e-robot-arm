#ifndef CORE_CONTROL_LOOP_INTERFACES_CONTROL_LOOP_HPP
#define CORE_CONTROL_LOOP_INTERFACES_CONTROL_LOOP_HPP

#include <cstdint>

namespace arm
{
    class ControlLoop
    {
    public:
        ControlLoop() = default;
        ControlLoop(const ControlLoop& other) = delete;
        ControlLoop& operator=(const ControlLoop& other) = delete;

        virtual void Start() = 0;
        virtual void Stop() = 0;
        virtual bool Running() const = 0;
        virtual uint32_t OverrunCount() const = 0;

    protected:
        ~ControlLoop() = default;
    };
}

#endif
