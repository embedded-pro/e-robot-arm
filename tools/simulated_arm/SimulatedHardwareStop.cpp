#include "tools/simulated_arm/SimulatedHardwareStop.hpp"

namespace simulation
{
    bool SimulatedHardwareStop::Open() const
    {
        return open;
    }

    void SimulatedHardwareStop::Set(bool newOpen)
    {
        if (newOpen == open)
            return;

        open = newOpen;
        NotifyObservers([newOpen](auto& observer)
            {
                observer.OnHardwareStopChanged(newOpen);
            });
    }
}
