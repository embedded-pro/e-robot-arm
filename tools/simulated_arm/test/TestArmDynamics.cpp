#include "tools/simulated_arm/ArmDynamics.hpp"
#include "gmock/gmock.h"

namespace
{
    constexpr float period{ 1.0e-3f };

    simulation::PlantConfig Geared()
    {
        simulation::PlantConfig config{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            config.transmissions[joint].ratio = 10.0f;
            config.rotorInertia[joint] = 1.0e-4f;
        }
        return config;
    }

    arm::JointPositions Pose()
    {
        arm::JointPositions pose{};
        pose[1] = foc::Radians{ 0.4f };
        pose[2] = foc::Radians{ -0.6f };
        return pose;
    }

    class ArmDynamicsTest
        : public testing::Test
    {
    public:
        ArmDynamicsTest()
        {
            dynamics.SetState(Pose(), arm::JointSpeeds{});
        }

        simulation::DrivingTorques HoldingTorques() const
        {
            const auto gravity = model.InverseDynamics(Pose(), arm::JointSpeeds{}, arm::JointAccelerations{});
            simulation::DrivingTorques torques{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                torques[joint] = foc::NewtonMeter{ gravity[joint].Value() / config.transmissions[joint].Gain() };
            return torques;
        }

        float Armature(std::size_t joint) const
        {
            const auto ratio = config.transmissions[joint].ratio;
            return config.rotorInertia[joint] * ratio * ratio;
        }

        simulation::PlantConfig config{ Geared() };
        arm::ArmModel model{ config.geometry, config.mass, arm::JointLimits{} };
        simulation::ArmDynamics dynamics{ config };
    };
}

TEST_F(ArmDynamicsTest, a_driven_joint_accelerates_by_its_torque_over_the_links_and_the_reflected_rotor)
{
    simulation::DrivingTorques torques{};
    torques[0] = foc::NewtonMeter{ 0.05f };

    dynamics.Step(torques, arm::JointTorques{}, period);

    const auto inertia = model.JointMassMatrix(Pose()).at(0, 0) + Armature(0);
    EXPECT_NEAR(10.0f * 0.05f / inertia * period, dynamics.Speeds()[0].Value(), 1.0e-5f);
    for (std::size_t joint = 1; joint != arm::jointCount; ++joint)
        EXPECT_EQ(0.0f, dynamics.Speeds()[joint].Value());
}

TEST_F(ArmDynamicsTest, holding_torques_keep_the_arm_still_and_a_torque_on_one_joint_moves_the_joints_coupled_to_it)
{
    auto torques = HoldingTorques();
    dynamics.Step(torques, arm::JointTorques{}, period);
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(0.0f, dynamics.Speeds()[joint].Value(), 1.0e-5f);

    dynamics.SetState(Pose(), arm::JointSpeeds{});
    torques[1] = foc::NewtonMeter{ torques[1]->Value() + 0.1f };
    dynamics.Step(torques, arm::JointTorques{}, period);

    EXPECT_GT(std::abs(dynamics.Speeds()[1].Value()), 1.0e-4f);
    EXPECT_GT(std::abs(dynamics.Speeds()[2].Value()), 1.0e-5f);
}

TEST_F(ArmDynamicsTest, an_external_torque_acts_on_the_joint_side)
{
    simulation::DrivingTorques torques{};
    torques[0] = foc::NewtonMeter{ 0.0f };
    arm::JointTorques external{};
    external[0] = foc::NewtonMeter{ 0.5f };

    dynamics.Step(torques, external, period);

    const auto inertia = model.JointMassMatrix(Pose()).at(0, 0) + Armature(0);
    EXPECT_NEAR(0.5f / inertia * period, dynamics.Speeds()[0].Value(), 1.0e-5f);
}

TEST_F(ArmDynamicsTest, a_held_joint_stops_and_keeps_its_position)
{
    arm::JointSpeeds speeds{};
    speeds[0] = foc::RadiansPerSecond{ 2.0f };
    dynamics.SetState(Pose(), speeds);

    dynamics.Step(simulation::DrivingTorques{}, arm::JointTorques{}, period);

    EXPECT_EQ(0.0f, dynamics.Speeds()[0].Value());
    EXPECT_NEAR(Pose()[0].Value(), dynamics.Positions()[0].Value(), 1.0e-6f);
}

TEST_F(ArmDynamicsTest, coulomb_friction_holds_a_joint_it_outweighs_and_stops_a_moving_one_without_reversing_it)
{
    auto config = Geared();
    config.coulombFriction[0] = 0.01f;
    simulation::ArmDynamics withFriction{ config };
    withFriction.SetState(Pose(), arm::JointSpeeds{});
    simulation::DrivingTorques torques{};
    torques[0] = foc::NewtonMeter{ 0.005f };

    withFriction.Step(torques, arm::JointTorques{}, period);
    EXPECT_EQ(0.0f, withFriction.Speeds()[0].Value());

    arm::JointSpeeds speeds{};
    speeds[0] = foc::RadiansPerSecond{ 0.1f };
    withFriction.SetState(Pose(), speeds);
    torques[0] = foc::NewtonMeter{ 0.0f };
    for (int step = 0; step != 2000; ++step)
    {
        withFriction.Step(torques, arm::JointTorques{}, period);
        EXPECT_GE(withFriction.Speeds()[0].Value(), 0.0f);
    }
    EXPECT_EQ(0.0f, withFriction.Speeds()[0].Value());
}

TEST_F(ArmDynamicsTest, a_joint_alone_has_its_diagonal_inertia_and_coupled_joints_seem_lighter)
{
    simulation::DrivingTorques alone{};
    alone[1] = foc::NewtonMeter{ 0.0f };
    const auto mass = model.JointMassMatrix(Pose());
    EXPECT_NEAR(mass.at(1, 1) + Armature(1), dynamics.ApparentInertias(alone)[1], 1.0e-4f);

    const auto coupled = dynamics.ApparentInertias(HoldingTorques());
    EXPECT_LT(coupled[1], mass.at(1, 1) + Armature(1));
    EXPECT_GT(coupled[1], 0.0f);
}
