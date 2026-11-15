#include "tools/virtual_arm/ArmCoupling.hpp"

namespace virtual_arm
{
    ArmCoupling::ArmCoupling(const arm::ArmConfig& config)
        : model(config.geometry, config.mass, config.positionLimits)
        , transmissions(config.transmissions)
    {}

    MotorTorques ArmCoupling::ShaftTorques(const MotorStates& motors, const arm::JointTorques& collision) const
    {
        arm::JointPositions positions{};
        arm::JointSpeeds speeds{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            positions[joint] = transmissions[joint].JointPosition(foc::Radians{ motors[joint].angle });
            speeds[joint] = transmissions[joint].JointSpeed(foc::RadiansPerSecond{ motors[joint].speed });
        }

        const auto held = model.InverseDynamics(positions, speeds, arm::JointAccelerations{});
        MotorTorques torques{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            torques[joint] = (collision[joint].Value() - held[joint].Value()) / transmissions[joint].Gain();
        return torques;
    }
}
