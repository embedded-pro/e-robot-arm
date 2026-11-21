#pragma once

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
