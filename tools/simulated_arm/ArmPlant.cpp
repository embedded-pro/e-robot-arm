#include "tools/simulated_arm/ArmPlant.hpp"
#include <algorithm>

namespace simulation
{
    namespace
    {
        constexpr float stepSeconds{ std::chrono::duration<float>(ArmPlant::step).count() };
    }

    ArmPlant::ArmPlant(const PlantConfig& config, const std::array<SimulatedJoint*, arm::jointCount>& joints)
        : transmissions(config.transmissions)
        , joints(joints)
        , dynamics(config)
    {
        arm::JointPositions positions{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            positions[joint] = transmissions[joint].JointPosition(joints[joint]->Position());
        dynamics.SetState(positions, arm::JointSpeeds{});
    }

    void ArmPlant::Advance()
    {
        const auto now = infra::Now();
        if (simulated + step > now)
            return;

        DrivingTorques torques{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            torques[joint] = joints[joint]->DrivingTorque();

        TakeKinematicJoints(torques);
        const auto driven = std::ranges::any_of(torques, [](const std::optional<foc::NewtonMeter>& torque)
            {
                return torque.has_value();
            });
        if (!driven || now - simulated > longestCatchUp)
            simulated = driven ? now - longestCatchUp : now;
        for (; simulated + step <= now; simulated += step)
            dynamics.Step(torques, external, stepSeconds);
    }

    void ArmPlant::Push(std::size_t joint, foc::NewtonMeter torque)
    {
        external[joint] = torque;
    }

    foc::Radians ArmPlant::MotorPosition(std::size_t joint) const
    {
        return transmissions[joint].MotorPosition(dynamics.Positions()[joint]);
    }

    foc::RadiansPerSecond ArmPlant::MotorSpeed(std::size_t joint) const
    {
        return transmissions[joint].MotorSpeed(dynamics.Speeds()[joint]);
    }

    void ArmPlant::TakeKinematicJoints(const DrivingTorques& torques)
    {
        auto positions = dynamics.Positions();
        auto speeds = dynamics.Speeds();
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            if (!torques[joint].has_value())
            {
                joints[joint]->Settle();
                positions[joint] = transmissions[joint].JointPosition(joints[joint]->Position());
                speeds[joint] = foc::RadiansPerSecond{ 0.0f };
            }
        dynamics.SetState(positions, speeds);
    }
}
