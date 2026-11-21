#pragma once

#include "infra/util/Observer.hpp"

namespace arm
{
    class HardwareStop;

    class HardwareStopObserver
        : public infra::Observer<HardwareStopObserver, HardwareStop>
    {
    public:
        using infra::Observer<HardwareStopObserver, HardwareStop>::Observer;

        virtual void OnHardwareStopChanged(bool open) = 0;

    protected:
        ~HardwareStopObserver() = default;
    };

    class HardwareStop
        : public infra::Subject<HardwareStopObserver>
    {
    public:
        HardwareStop() = default;
        HardwareStop(const HardwareStop& other) = delete;
        HardwareStop& operator=(const HardwareStop& other) = delete;

        virtual bool Open() const = 0;

    protected:
        ~HardwareStop() = default;
    };
}
