#ifndef CORE_SAFETY_INTERFACES_SAFETY_SUPERVISOR_HPP
#define CORE_SAFETY_INTERFACES_SAFETY_SUPERVISOR_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "e-foc-protocol/foc/Units.hpp"
#include <array>

namespace arm
{
    class SafetySupervisor
        : public JointBus
    {
    public:
        virtual bool Tripped() const = 0;
        virtual bool Rearm() = 0;

    protected:
        ~SafetySupervisor() = default;
    };
}

#endif
