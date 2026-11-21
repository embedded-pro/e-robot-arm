#include "core/arm_model/JointFrameMap.hpp"
#include "gmock/gmock.h"

namespace
{
    constexpr float tolerance{ 1.0e-6f };

    class JointFrameMapTest
        : public testing::Test
    {
    public:
        arm::JointFrameMap map{ { 1, -1, 1, -1, 1, -1 }, { 0.1f, -0.2f, 0.3f, 0.0f, 0.2f, -0.1f } };
        const arm::JointPositions positions{ foc::Radians{ 0.5f }, foc::Radians{ -0.4f }, foc::Radians{ 0.3f }, foc::Radians{ 0.2f },
            foc::Radians{ -0.1f }, foc::Radians{ 0.6f } };
    };
}

TEST(JointVectorTest, copies_quantities_into_a_vector_and_back)
{
    const arm::JointSpeeds speeds{ foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecond{ -2.0f }, foc::RadiansPerSecond{ 3.0f },
        foc::RadiansPerSecond{ -4.0f }, foc::RadiansPerSecond{ 5.0f }, foc::RadiansPerSecond{ -6.0f } };

    const auto vector = arm::ToVector(speeds);
    const auto back = arm::FromVector<foc::RadiansPerSecond>(vector);

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_FLOAT_EQ(vector[joint], speeds[joint].Value());
        EXPECT_FLOAT_EQ(back[joint].Value(), speeds[joint].Value());
    }
}

TEST_F(JointFrameMapTest, angles_are_the_signed_positions_less_the_offsets)
{
    const arm::JointVector expected{ 0.4f, 0.6f, 0.0f, -0.2f, -0.3f, -0.5f };

    const auto angles = map.Angles(positions);

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(angles[joint], expected[joint], tolerance);
}

TEST_F(JointFrameMapTest, positions_undo_the_angles)
{
    const auto back = map.Positions(map.Angles(positions));

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        EXPECT_NEAR(back[joint].Value(), positions[joint].Value(), tolerance);
}

TEST_F(JointFrameMapTest, rates_and_torques_only_take_the_signs)
{
    const arm::JointTorques torques{ foc::NewtonMeter{ 1.0f }, foc::NewtonMeter{ 2.0f }, foc::NewtonMeter{ 3.0f }, foc::NewtonMeter{ 4.0f },
        foc::NewtonMeter{ 5.0f }, foc::NewtonMeter{ 6.0f } };
    const arm::JointVector expected{ 1.0f, -2.0f, 3.0f, -4.0f, 5.0f, -6.0f };

    const auto model = map.ToModel(torques);
    const auto back = map.ToJoint<foc::NewtonMeter>(model);

    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
    {
        EXPECT_FLOAT_EQ(model[joint], expected[joint]);
        EXPECT_FLOAT_EQ(back[joint].Value(), torques[joint].Value());
    }
}

TEST_F(JointFrameMapTest, flip_columns_and_congruence_are_products_with_the_sign_matrix)
{
    arm::JointMatrix matrix{};
    for (std::size_t row = 0; row != arm::jointCount; ++row)
        for (std::size_t column = 0; column != arm::jointCount; ++column)
            matrix.at(row, column) = static_cast<float>(10 * row + column + 1);
    arm::JointMatrix signs{};
    for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        signs.at(joint, joint) = map.Flip(arm::JointVector{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f })[joint];

    const auto columns = map.FlipColumns(matrix);
    const auto congruent = map.Congruence(matrix);
    const auto expectedColumns = matrix * signs;
    const auto expectedCongruent = signs * matrix * signs;

    for (std::size_t row = 0; row != arm::jointCount; ++row)
        for (std::size_t column = 0; column != arm::jointCount; ++column)
        {
            EXPECT_FLOAT_EQ(columns.at(row, column), expectedColumns.at(row, column));
            EXPECT_FLOAT_EQ(congruent.at(row, column), expectedCongruent.at(row, column));
        }
}
