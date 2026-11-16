#ifndef TOOLS_ARM_HOST_VIRTUAL_MOTORS_HPP
#define TOOLS_ARM_HOST_VIRTUAL_MOTORS_HPP

#include "core/arm_model/ArmModel.hpp"
#include "tools/virtual_arm/MotorReport.hpp"
#include <cstddef>
#include <optional>
#include <string>

namespace arm_host
{
    struct VirtualMotorsStatus
    {
        virtual_arm::MotorReports motors{};
        std::string folder;
        std::array<bool, arm::jointCount> silenced{};
        arm::JointTorques torques{};

        friend bool operator==(const VirtualMotorsStatus& lhs, const VirtualMotorsStatus& rhs) = default;
    };

    class VirtualMotors
    {
    public:
        virtual VirtualMotorsStatus Status() const = 0;
        virtual std::optional<std::string> Restart(std::size_t joint) = 0;
        virtual void Silence(std::size_t joint, bool silent) = 0;
        virtual void Collide(std::size_t joint, foc::NewtonMeter torque) = 0;
        virtual std::optional<std::string> PowerOff(std::size_t joint) = 0;

    protected:
        ~VirtualMotors() = default;
    };
}

#endif
