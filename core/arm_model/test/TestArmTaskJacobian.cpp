#include "core/arm_model/ArmTaskJacobian.hpp"
#include "gmock/gmock.h"
#include <numbers>

namespace
{
    constexpr float pi{ std::numbers::pi_v<float> };

    arm::JointLimits FullTurn()
    {
        arm::JointLimits limits{};
        for (auto& limit : limits)
            limit = { foc::Radians{ -pi }, foc::Radians{ pi } };
        return limits;
    }

    arm::ArmGeometry SignedGeometry()
    {
        auto geometry = arm::ReferenceGeometry();
        geometry.opw.a2 = 0.03f;
        geometry.opw.b = 0.04f;
        geometry.opw.signs = { 1, -1, 1, -1, 1, -1 };
        geometry.opw.offsets = { 0.1f, -0.2f, 0.3f, 0.0f, 0.2f, -0.1f };
        geometry.tool = { 0.01f, 0.0f, 0.05f, 0.0f, 0.0f, 0.0f };
        geometry.base = { 0.0f, 0.0f, 0.0f, 0.2f, 0.3f, 0.0f };
        return geometry;
    }

    class ArmTaskJacobianTest
        : public testing::Test
    {
    public:
        arm::ArmModel model{ SignedGeometry(), arm::ReferenceMass(), FullTurn() };
        arm::ArmTaskJacobian provider{ model };
        const arm::JointPositions joints{ foc::Radians{ 0.3f }, foc::Radians{ 0.4f }, foc::Radians{ -0.6f }, foc::Radians{ 0.5f }, foc::Radians{ 0.8f },
            foc::Radians{ -0.2f } };
        const arm::ArmModel::JointVector angles{ model.ModelVector(joints) };
    };
}

TEST_F(ArmTaskJacobianTest, gives_the_tool_pose_and_jacobian_in_the_dynamics_coordinates)
{
    const auto pose = provider.ToolPose(angles);
    const auto expected = model.ToolPose(joints);
    for (std::size_t axis = 0; axis != 3; ++axis)
    {
        EXPECT_NEAR(expected.translation[axis], pose.translation[axis], 1.0e-6f);
        for (std::size_t column = 0; column != 3; ++column)
            EXPECT_NEAR(expected.rotation.at(axis, column), pose.rotation.at(axis, column), 1.0e-6f);
    }

    const auto jacobian = provider.Jacobian(angles);
    const auto jointJacobian = model.ToolJacobian(joints);
    const std::array<float, arm::jointCount> signs{ 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f };
    for (std::size_t row = 0; row != 6; ++row)
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            EXPECT_NEAR(signs[joint] * jointJacobian.at(row, joint), jacobian.at(row, joint), 1.0e-6f) << row << ", " << joint;
}

TEST_F(ArmTaskJacobianTest, the_bias_acceleration_is_the_jacobians_rate_along_the_speeds_times_the_speeds)
{
    arm::ArmModel::JointVector speeds{};
    const std::array<float, arm::jointCount> rates{ 0.5f, -0.3f, 0.8f, 1.0f, -0.6f, 0.4f };
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        speeds.at(joint, 0) = rates[joint];

    constexpr float step{ 1.0e-3f };
    const auto ahead = provider.Jacobian(angles + speeds * step);
    const auto behind = provider.Jacobian(angles - speeds * step);
    const auto bias = provider.BiasAcceleration(angles, speeds);

    for (std::size_t row = 0; row != 6; ++row)
    {
        float expected{ 0.0f };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            expected += (ahead.at(row, joint) - behind.at(row, joint)) / (2.0f * step) * rates[joint];
        EXPECT_NEAR(expected, bias.at(row, 0), 2.0e-3f) << row;
    }
}
