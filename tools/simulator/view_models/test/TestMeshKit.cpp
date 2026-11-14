#include "tools/simulator/view_models/MeshKit.hpp"
#include "gmock/gmock.h"
#include <cmath>

namespace
{
    using simulator::mesh::Mesh;
    using simulator::mesh::Vec3;

    float Length(const Vec3& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    Vec3 Cross(const Vec3& a, const Vec3& b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    class MeshKitTest
        : public testing::Test
    {
    public:
        void ExpectOutwardUnitNormals(const Mesh& mesh, const Vec3& centre) const
        {
            const auto& vertices = mesh.Vertices();
            const auto& indices = mesh.Indices();
            for (std::size_t triangle = 0; triangle != mesh.Triangles(); ++triangle)
            {
                const auto& a = vertices[indices[3 * triangle]];
                const auto& b = vertices[indices[3 * triangle + 1]];
                const auto& c = vertices[indices[3 * triangle + 2]];
                const auto face = Cross({ b.position.x - a.position.x, b.position.y - a.position.y, b.position.z - a.position.z },
                    { c.position.x - a.position.x, c.position.y - a.position.y, c.position.z - a.position.z });
                if (Length(face) < 1.0e-9f)
                    continue;
                const Vec3 outward{ a.position.x - centre.x, a.position.y - centre.y, a.position.z - centre.z };
                EXPECT_GT(face.x * outward.x + face.y * outward.y + face.z * outward.z, 0.0f) << "winding of triangle " << triangle;
                EXPECT_GT(a.normal.x * face.x + a.normal.y * face.y + a.normal.z * face.z, 0.0f) << "normal of triangle " << triangle;
                EXPECT_NEAR(1.0f, Length(a.normal), 1.0e-5f);
            }
        }
    };
}

TEST_F(MeshKitTest, a_lathe_turns_a_profile_into_a_closed_solid_facing_outwards)
{
    const auto cylinder = simulator::mesh::Lathe({ { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 2.0f }, { 0.0f, 2.0f } }, 8);

    EXPECT_EQ(3u * 8u * 2u, cylinder.Triangles());
    const auto bounds = cylinder.Extent();
    EXPECT_NEAR(-1.0f, bounds.lowest.x, 1.0e-5f);
    EXPECT_NEAR(2.0f, bounds.highest.z, 1.0e-5f);
    ExpectOutwardUnitNormals(cylinder, { 0.0f, 0.0f, 1.0f });
}

TEST_F(MeshKitTest, an_extrusion_has_flat_faces_hard_corners_and_smooth_rounded_ones)
{
    const auto bar = simulator::mesh::Extrude(simulator::mesh::RoundedRectangle(2.0f, 1.0f, 0.0f, 1), -0.5f, 0.5f);
    EXPECT_EQ(4u * 2u + 4u * 2u, bar.Triangles());
    ExpectOutwardUnitNormals(bar, {});
    for (const auto& vertex : bar.Vertices())
        EXPECT_TRUE(std::abs(vertex.normal.x) == 1.0f || std::abs(vertex.normal.y) == 1.0f || std::abs(vertex.normal.z) == 1.0f);

    const auto rounded = simulator::mesh::Extrude(simulator::mesh::RoundedRectangle(2.0f, 1.0f, 0.3f, 6), 0.0f, 1.0f);
    ExpectOutwardUnitNormals(rounded, { 0.0f, 0.0f, 0.5f });
    const auto diagonal = std::ranges::count_if(rounded.Vertices(), [](const simulator::mesh::Vertex& vertex)
        {
            return std::abs(vertex.normal.x) > 0.1f && std::abs(vertex.normal.y) > 0.1f;
        });
    EXPECT_GT(diagonal, 0);
}

TEST_F(MeshKitTest, outlines_wind_counter_clockwise_around_their_centre)
{
    for (const auto& outline : { simulator::mesh::Stadium(3.0f, 1.0f, 8), simulator::mesh::RegularPolygon(6, 1.0f), simulator::mesh::RoundedRectangle(1.0f, 2.0f, 0.2f, 4) })
    {
        float area{ 0.0f };
        for (std::size_t index = 0; index != outline.size(); ++index)
        {
            const auto& p = outline[index];
            const auto& q = outline[(index + 1) % outline.size()];
            area += p[0] * q[1] - q[0] * p[1];
        }
        EXPECT_GT(area, 0.0f);
    }
    EXPECT_EQ(6u, simulator::mesh::RegularPolygon(6, 1.0f).size());
}

TEST_F(MeshKitTest, parts_are_placed_along_an_axis_with_their_normals_turned)
{
    const auto disc = simulator::mesh::Lathe({ { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 0.1f }, { 0.0f, 0.1f } }, 12);
    Mesh assembly;
    assembly.Add(disc, simulator::mesh::Along({ 5.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }));

    const auto bounds = assembly.Extent();
    EXPECT_NEAR(0.0f, bounds.lowest.y, 1.0e-5f);
    EXPECT_NEAR(0.1f, bounds.highest.y, 1.0e-5f);
    EXPECT_NEAR(4.0f, bounds.lowest.x, 1.0e-5f);
    EXPECT_EQ(disc.Triangles(), assembly.Triangles());
    ExpectOutwardUnitNormals(assembly, { 5.0f, 0.05f, 0.0f });

    const auto frame = simulator::mesh::Along({}, { 0.0f, 0.0f, -1.0f });
    const auto z = frame.Rotate({ 0.0f, 0.0f, 1.0f });
    EXPECT_NEAR(-1.0f, z.z, 1.0e-6f);
    EXPECT_NEAR(1.0f, Length(Cross(frame.xAxis, frame.yAxis)), 1.0e-6f);
}
