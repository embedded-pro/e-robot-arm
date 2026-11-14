#include "tools/simulator/view_models/DesktopArm.hpp"
#include "gmock/gmock.h"

namespace
{
    using simulator::ArmFinish;

    class DesktopArmTest
        : public testing::Test
    {
    public:
        const simulator::mesh::Mesh& Of(std::size_t node, ArmFinish finish) const
        {
            return arm[node][static_cast<std::size_t>(finish)];
        }

        arm::ArmGeometry geometry{ arm::ReferenceGeometry() };
        float r{ simulator::PartThickness(geometry) };
        simulator::ArmMeshes arm{ simulator::BuildDesktopArm(geometry) };
    };
}

TEST_F(DesktopArmTest, every_moving_link_has_a_hub_in_the_accent_finish_and_the_driving_links_carry_motors)
{
    for (std::size_t node = 1; node != 6; ++node)
        EXPECT_FALSE(Of(node, ArmFinish::accent).Empty()) << node;
    for (std::size_t node : { 1u, 2u, 3u, 4u })
        EXPECT_FALSE(Of(node, ArmFinish::motor).Empty()) << node;
    EXPECT_FALSE(Of(0, ArmFinish::anodised).Empty());
    EXPECT_FALSE(Of(6, ArmFinish::steel).Empty());
}

TEST_F(DesktopArmTest, the_links_span_their_opw_lengths)
{
    const auto plates = Of(2, ArmFinish::aluminium).Extent();
    EXPECT_NEAR(-0.65f * r, plates.lowest.z, 1.0e-5f);
    EXPECT_NEAR(geometry.opw.c2 + 0.65f * r, plates.highest.z, 1.0e-5f);

    const auto turret = Of(1, ArmFinish::aluminium).Extent();
    EXPECT_NEAR(geometry.opw.c1 + 1.0f * r, turret.highest.z, 1.0e-5f);

    const auto flange = Of(6, ArmFinish::steel).Extent();
    EXPECT_NEAR(geometry.opw.c4 + 0.036f * r, flange.highest.z, 1.0e-5f);

    auto longer = geometry;
    longer.opw.c2 = 0.4f;
    const auto stretched = simulator::BuildDesktopArm(longer)[2][static_cast<std::size_t>(ArmFinish::aluminium)].Extent();
    EXPECT_NEAR(0.4f + 0.65f * simulator::PartThickness(longer), stretched.highest.z, 1.0e-5f);
}

TEST_F(DesktopArmTest, the_whole_arm_stays_within_its_triangle_budget)
{
    std::size_t triangles{ 0 };
    for (const auto& node : arm)
        for (const auto& part : node)
            triangles += part.Triangles();
    EXPECT_GT(triangles, 2000u);
    EXPECT_LT(triangles, 40000u);
}
