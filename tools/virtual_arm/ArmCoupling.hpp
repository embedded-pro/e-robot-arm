#ifndef TOOLS_VIRTUAL_ARM_ARM_COUPLING_HPP
#define TOOLS_VIRTUAL_ARM_ARM_COUPLING_HPP

#include "core/arm/ArmConfig.hpp"
#include "core/arm_model/ArmModel.hpp"
#include <array>

namespace virtual_arm
{
    struct MotorState
    {
        float angle{ 0.0f };
        float speed{ 0.0f };
    };

    using MotorStates = std::array<MotorState, arm::jointCount>;
    using MotorTorques = std::array<float, arm::jointCount>;

    class ArmCoupling
    {
    public:
        explicit ArmCoupling(const arm::ArmConfig& config);

        MotorTorques ShaftTorques(const MotorStates& motors, const arm::JointTorques& collision = {}) const;

    private:
        arm::ArmModel model;
        arm::JointTransmissions transmissions;
    };
}

#endif
