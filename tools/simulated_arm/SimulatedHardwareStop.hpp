#ifndef TOOLS_SIMULATED_ARM_SIMULATED_HARDWARE_STOP_HPP
#define TOOLS_SIMULATED_ARM_SIMULATED_HARDWARE_STOP_HPP

#include "core/safety/interfaces/HardwareStop.hpp"

namespace simulation
{
    class SimulatedHardwareStop
        : public arm::HardwareStop
    {
    public:
        bool Open() const override;
        void Set(bool open);

    private:
        bool open{ false };
    };
}

#endif
