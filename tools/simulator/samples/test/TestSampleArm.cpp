#include "tools/simulator/samples/SampleArm.hpp"
#include "gmock/gmock.h"
#include <fstream>
#include <sstream>

namespace
{
    constexpr std::size_t triangleBudget{ 50000 };

#ifdef SAMPLE_MESHES_DIR
    std::string Committed(const std::string& name)
    {
        std::ifstream stream{ std::string{ SAMPLE_MESHES_DIR } + "/" + name, std::ios::binary };
        std::ostringstream text;
        text << stream.rdbuf();
        return text.str();
    }
#endif

    ui::stage::ShadedMesh Triangle()
    {
        ui::stage::ShadedMesh mesh;
        mesh.vertices = { { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } }, { { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } }, { { 0.0f, 2.0f, -1.0f }, { 0.0f, 0.0f, 1.0f } } };
        mesh.indices = { 0, 1, 2 };
        return mesh;
    }
}

#ifdef SAMPLE_MESHES_DIR
TEST(SampleArmTest, the_committed_sample_files_are_the_ones_the_tool_writes)
{
    for (const auto& file : samples::SampleArmFiles())
        EXPECT_TRUE(file.text == Committed(file.name)) << file.name << " is stale: run e-robot-arm-sample-meshes tools/simulator/samples/meshed-arm";
}
#endif

TEST(SampleArmTest, the_profile_names_a_mesh_for_the_base_and_each_link)
{
    const auto profile = samples::SampleProfile();
    for (std::size_t node = 0; node != arm::meshCount; ++node)
        EXPECT_EQ(std::string{ arm::meshNames[node] } + ".gltf", std::string(profile.meshes[node].begin(), profile.meshes[node].end()));
}

TEST(SampleArmTest, each_node_keeps_to_the_simulator_triangle_budget)
{
    const auto geometry = samples::SampleProfile().config.geometry;
    for (std::size_t node = 0; node != arm::meshCount; ++node)
    {
        const auto primitives = samples::SampleNodeMeshes(geometry, node);
        std::size_t triangles{ 0 };
        for (const auto& primitive : primitives)
            triangles += primitive.mesh.Triangles();
        EXPECT_FALSE(primitives.empty()) << arm::meshNames[node];
        EXPECT_GT(triangles, 0u) << arm::meshNames[node];
        EXPECT_LE(triangles, triangleBudget) << arm::meshNames[node];
    }
}

TEST(GltfWriterTest, writes_positions_normals_indices_and_the_material_of_each_primitive)
{
    const auto text = samples::WriteGltf("link", { { { "painted", { 0.5f, 0.25f, 0.0f }, 0.0f, 0.45f }, Triangle() } });

    EXPECT_THAT(text, testing::HasSubstr(R"("version": "2.0")"));
    EXPECT_THAT(text, testing::HasSubstr(R"("POSITION": 0, "NORMAL": 1 }, "indices": 2, "material": 0, "mode": 4)"));
    EXPECT_THAT(text, testing::HasSubstr(R"("count": 3, "type": "VEC3", "min": [0, 0, -1], "max": [1, 2, 0])"));
    EXPECT_THAT(text, testing::HasSubstr(R"("componentType": 5125, "count": 3, "type": "SCALAR")"));
    EXPECT_THAT(text, testing::HasSubstr(R"("baseColorFactor": [0.5, 0.25, 0, 1], "metallicFactor": 0, "roughnessFactor": 0.45)"));
    EXPECT_THAT(text, testing::HasSubstr(R"("byteLength": 84, "uri": "data:application/octet-stream;base64,)"));
}

TEST(GltfWriterTest, pads_the_embedded_buffer_to_whole_base64_groups)
{
    auto mesh = Triangle();
    mesh.vertices.resize(1);
    mesh.indices = { 0 };
    const auto text = samples::WriteGltf("point", { { {}, mesh } });

    EXPECT_THAT(text, testing::HasSubstr(R"("byteLength": 28, "uri": "data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAAAAAAAAAAAAAIA/AAAAAA==")"));
}
