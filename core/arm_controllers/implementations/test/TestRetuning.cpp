#include "core/arm_controllers/implementations/CartesianImpedance.hpp"
#include "core/arm_controllers/implementations/ComputedTorque.hpp"
#include "core/arm_controllers/implementations/GravityCompensatedPd.hpp"
#include "core/arm_controllers/implementations/GravityPid.hpp"
#include "core/arm_controllers/implementations/JointImpedance.hpp"
#include "core/arm_controllers/implementations/OperationalSpace.hpp"
#include "core/arm_controllers/implementations/SlotineLi.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    using namespace std::chrono_literals;

    arm::JointLimits WideLimits()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -3.0f }, foc::Radians{ 3.0f } };
        return limits;
    }

    class RetuningTest
        : public testing::Test
    {
    public:
        RetuningTest()
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                measured.positions[joint] = foc::Radians{ 0.1f + 0.1f * static_cast<float>(joint) };
                measured.speeds[joint] = foc::RadiansPerSecond{ 0.05f * static_cast<float>(joint) };
                target.positions[joint] = foc::Radians{ measured.positions[joint].Value() + 0.03f };
                target.speeds[joint] = foc::RadiansPerSecond{ 0.2f };
            }
        }

        template<class Controller>
        void ExpectSameAsBuilt(Controller& retuned, Controller& built)
        {
            retuned.Reset(measured, arm::AtRest(measured.positions));
            built.Reset(measured, arm::AtRest(measured.positions));
            for (int step = 0; step != 3; ++step)
            {
                const auto expected = built.Step(measured, target);
                const auto actual = retuned.Step(measured, target);
                for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                    EXPECT_NEAR(expected.torques[joint].Value(), actual.torques[joint].Value(), 1.0e-3f) << "joint " << joint + 1 << ", step " << step;
            }
        }

        arm::ArmModel model{ arm::ReferenceGeometry(), arm::ReferenceMass(), WideLimits() };
        arm::DrivenArm drivenArm{ model, arm::JointInertias{ 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f } };
        arm::MeasuredJoints measured{};
        arm::JointReference target{};
        const arm::BandwidthTuning slow{ 10.0f, 0.5f };
        const arm::BandwidthTuning fast{ 40.0f, 0.9f };
    };
}

TEST_F(RetuningTest, gravity_pd_takes_a_new_bandwidth)
{
    arm::GravityCompensatedPd retuned{ drivenArm, slow };
    arm::GravityCompensatedPd built{ drivenArm, fast };
    retuned.SetTuning(fast);
    ExpectSameAsBuilt(retuned, built);
}

TEST_F(RetuningTest, computed_torque_takes_a_new_bandwidth)
{
    arm::ComputedTorque retuned{ drivenArm, slow };
    arm::ComputedTorque built{ drivenArm, fast };
    retuned.SetTuning(fast);
    ExpectSameAsBuilt(retuned, built);
}

TEST_F(RetuningTest, gravity_pid_takes_a_new_bandwidth)
{
    arm::GravityPid retuned{ drivenArm, slow, 10ms };
    arm::GravityPid built{ drivenArm, fast, 10ms };
    retuned.SetTuning(fast);
    ExpectSameAsBuilt(retuned, built);
}

TEST_F(RetuningTest, joint_impedance_takes_new_stiffnesses_and_keeps_its_share)
{
    arm::JointStiffnesses stiffness{};
    stiffness.fill(150.0f);
    const arm::JointImpedanceTuning stiff{ stiffness, 0.9f };
    arm::JointImpedance retuned{ drivenArm, arm::JointImpedanceTuning{} };
    arm::JointImpedance built{ drivenArm, stiff };
    ASSERT_TRUE(retuned.SetStiffnessShare(0.5f));
    ASSERT_TRUE(built.SetStiffnessShare(0.5f));
    retuned.SetTuning(stiff);
    EXPECT_FLOAT_EQ(0.5f, retuned.StiffnessShare());
    ExpectSameAsBuilt(retuned, built);
}

TEST_F(RetuningTest, cartesian_impedance_takes_new_stiffnesses)
{
    const arm::CartesianImpedanceTuning stiff{ 120.0f, 0.2f, 0.9f };
    arm::CartesianImpedance retuned{ drivenArm, arm::CartesianImpedanceTuning{} };
    arm::CartesianImpedance built{ drivenArm, stiff };
    retuned.SetTuning(stiff);
    ExpectSameAsBuilt(retuned, built);
}

TEST_F(RetuningTest, operational_space_takes_a_new_bandwidth_and_singularity_damping)
{
    const arm::OperationalSpaceTuning fastTask{ 40.0f, 0.9f, 0.05f };
    arm::OperationalSpace retuned{ drivenArm, arm::OperationalSpaceTuning{}, 0.0f };
    arm::OperationalSpace built{ drivenArm, fastTask, 0.0f };
    retuned.SetTuning(fastTask);
    ExpectSameAsBuilt(retuned, built);
}

TEST_F(RetuningTest, slotine_li_takes_new_gains_and_keeps_what_it_learnt)
{
    const arm::SlotineLiTuning sharp{ 20.0f, 30.0f, 50.0f };
    arm::SlotineLi retuned{ drivenArm, arm::SlotineLiTuning{}, 10ms };
    retuned.Reset(measured, arm::AtRest(measured.positions));
    retuned.Step(measured, target);
    const auto learnt = retuned.Estimate();

    retuned.SetTuning(sharp);

    for (std::size_t index = 0; index != arm::SlotineLi::Parameters::rows; ++index)
        EXPECT_FLOAT_EQ(learnt.at(index, 0), retuned.Estimate().at(index, 0));
    arm::SlotineLi built{ drivenArm, sharp, 10ms };
    retuned.ResetEstimate();
    ExpectSameAsBuilt(retuned, built);
}
