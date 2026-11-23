#include "tools/simulator/samples/SampleArm.hpp"
#include "infra/stream/StdStringOutputStream.hpp"
#include "tools/arm_host/DefaultArmConfig.hpp"
#include "ui/stage/Primitives.hpp"
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <numbers>

namespace samples
{
    namespace
    {
        using ui::scene::Matrix3;
        using ui::scene::Transform3;
        using ui::stage::Mesh;
        using Vec3 = ui::scene::Vector3;

        enum class Finish : uint8_t
        {
            painted,
            cast,
            anodised,
            rubber,
            steel
        };

        constexpr std::size_t finishCount{ 5 };
        using NodeParts = std::array<Mesh, finishCount>;

        constexpr ui::stage::Tessellation tessellation{ 48 };
        constexpr float creaseAngleDegrees{ 35.0f };
        const Vec3 up{ 0.0f, 0.0f, 1.0f };
        const Vec3 side{ 0.0f, 1.0f, 0.0f };
        const Vec3 otherSide{ 0.0f, -1.0f, 0.0f };

        float Linear(float srgb)
        {
            return srgb <= 0.04045f ? srgb / 12.92f : std::pow((srgb + 0.055f) / 1.055f, 2.4f);
        }

        GltfMaterial MaterialOf(Finish finish)
        {
            struct Recipe
            {
                const char* name;
                uint32_t srgb;
                float metallic;
                float roughness;
            };

            constexpr std::array<Recipe, finishCount> recipes{ {
                { "painted", 0xf28c28, 0.0f, 0.4f },
                { "cast", 0x3c4043, 0.6f, 0.55f },
                { "anodised", 0x9aa0a6, 1.0f, 0.35f },
                { "rubber", 0x1d1f21, 0.0f, 0.8f },
                { "steel", 0xc8ccd0, 1.0f, 0.25f },
            } };
            const auto& recipe = recipes[static_cast<std::size_t>(finish)];
            const auto channel = [&recipe](uint32_t shift)
            {
                return Linear(static_cast<float>((recipe.srgb >> shift) & 0xff) / 255.0f);
            };
            return { recipe.name, { channel(16), channel(8), channel(0) }, recipe.metallic, recipe.roughness };
        }

        Transform3 At(const Vec3& origin)
        {
            return Transform3::Translation(origin);
        }

        Transform3 Along(const Vec3& origin, const Vec3& axis)
        {
            const auto z = ui::scene::Normalized(axis);
            const Vec3 helper = std::abs(z.z) < 0.9f ? Vec3{ 0.0f, 0.0f, 1.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
            const auto x = ui::scene::Normalized(ui::scene::Cross(helper, z));
            return Transform3{ Matrix3{ { x, ui::scene::Cross(z, x), z } }, origin };
        }

        Mesh Lathe(std::initializer_list<ui::stage::ProfilePoint> bottomToTop)
        {
            const std::vector<ui::stage::ProfilePoint> topToBottom(std::rbegin(bottomToTop), std::rend(bottomToTop));
            return ui::stage::MakeLathe(topToBottom, tessellation);
        }

        Mesh Disc(float radius, float bottom, float top, float chamfer)
        {
            return Lathe({ { 0.0f, bottom }, { radius, bottom }, { radius, top - chamfer }, { radius - chamfer, top }, { 0.0f, top } });
        }

        Mesh Hub(float radius, float length, float chamfer)
        {
            const auto h = length / 2.0f;
            return Lathe({ { 0.0f, -h }, { radius - chamfer, -h }, { radius, -h + chamfer }, { radius, h - chamfer }, { radius - chamfer, h }, { 0.0f, h } });
        }

        struct Of
        {
            NodeParts& node;
            Finish finish;

            void Add(const Mesh& part, const Transform3& frame = {}) const
            {
                node[static_cast<std::size_t>(finish)].Append(part, frame);
            }
        };

        void Plate(NodeParts& node, Finish finish, const Vec3& start, float length, float width, float thickness)
        {
            const Transform3 frame{ Matrix3{ { ui::scene::Cross(up, side), up, side } }, start };
            Of{ node, finish }.Add(ui::stage::MakeExtrusion(ui::stage::Stadium(length, width, 24), -thickness / 2.0f, thickness / 2.0f), frame);
        }

        void Base(NodeParts& node, float pedestal, float r)
        {
            Of{ node, Finish::cast }.Add(ui::stage::MakeExtrusion(ui::stage::RoundedRectangle(4.6f * r, 4.6f * r, 0.6f * r, 6), 0.0f, 0.25f * r));
            Of{ node, Finish::cast }.Add(Lathe({ { 0.0f, 0.25f * r }, { 1.6f * r, 0.25f * r }, { 1.5f * r, pedestal - 0.06f * r }, { 1.42f * r, pedestal }, { 0.0f, pedestal } }));
            const auto bolt = ui::stage::MakeExtrusion(ui::stage::RegularPolygon(6, 0.16f * r), 0.25f * r, 0.36f * r);
            for (const auto x : { -1.8f * r, 1.8f * r })
                for (const auto y : { -1.8f * r, 1.8f * r })
                    Of{ node, Finish::steel }.Add(bolt, At({ x, y, 0.0f }));
        }

        void Turret(NodeParts& node, const kinematics::OpwParameters<float>& p, float pedestal, float r)
        {
            const auto deck = pedestal + 0.15f * r;
            Of{ node, Finish::anodised }.Add(Disc(1.55f * r, pedestal, deck, 0.04f * r));
            Of{ node, Finish::painted }.Add(ui::stage::MakeExtrusion(ui::stage::RoundedRectangle(std::abs(p.a1) + 2.6f * r, 2.4f * r, 0.8f * r, 8), deck, p.c1), At({ p.a1 / 2.0f, p.b, 0.0f }));
            Of{ node, Finish::painted }.Add(Hub(1.2f * r, 2.6f * r, 0.12f * r), Along({ p.a1, p.b, p.c1 }, side));
            Of{ node, Finish::rubber }.Add(Disc(0.8f * r, 0.0f, 0.25f * r, 0.05f * r), Along({ p.a1, p.b - 1.3f * r, p.c1 }, otherSide));
        }

        void UpperArm(NodeParts& node, float c2, float r)
        {
            Plate(node, Finish::painted, {}, c2, 2.0f * r, 1.5f * r);
            Of{ node, Finish::anodised }.Add(Hub(0.9f * r, 1.9f * r, 0.08f * r), Along({ 0.0f, 0.0f, c2 }, side));
            Of{ node, Finish::rubber }.Add(Disc(0.6f * r, 0.0f, 0.2f * r, 0.04f * r), Along({ 0.0f, -0.95f * r, c2 }, otherSide));
        }

        void Forearm(NodeParts& node, const kinematics::OpwParameters<float>& p, float r)
        {
            const Vec3 reach{ p.a2, 0.0f, p.c3 };
            const auto length = std::max(std::sqrt(p.a2 * p.a2 + p.c3 * p.c3) - 0.95f * r, 0.1f * r);
            Of{ node, Finish::painted }.Add(Hub(1.05f * r, 1.5f * r, 0.1f * r), Along({}, side));
            Of{ node, Finish::painted }.Add(Lathe({ { 0.0f, 0.0f }, { 0.8f * r, 0.0f }, { 0.8f * r, 0.15f * length }, { 0.6f * r, length }, { 0.0f, length } }), Along({}, reach));
        }

        void WristRoll(NodeParts& node, float r)
        {
            Of{ node, Finish::cast }.Add(Lathe({ { 0.0f, -0.95f * r }, { 0.62f * r, -0.95f * r }, { 0.62f * r, -0.4f * r }, { 0.55f * r, -0.3f * r }, { 0.0f, -0.3f * r } }));
            for (const auto offset : { -0.5f * r, 0.5f * r })
                Plate(node, Finish::painted, { 0.0f, offset, -0.35f * r }, 0.35f * r, 1.0f * r, 0.14f * r);
        }

        void WristPitch(NodeParts& node, float c4, float r)
        {
            Of{ node, Finish::anodised }.Add(Hub(0.42f * r, 0.86f * r, 0.05f * r), Along({}, side));
            const auto top = std::max(c4 - 0.32f * r, 0.45f * r);
            Of{ node, Finish::painted }.Add(ui::stage::MakeExtrusion(ui::stage::RoundedRectangle(0.8f * r, 0.8f * r, 0.3f * r, 6), 0.3f * r, top));
        }

        void Flange(NodeParts& node, float c4, float r)
        {
            Of{ node, Finish::anodised }.Add(Lathe({ { 0.0f, c4 - 0.32f * r }, { 0.5f * r, c4 - 0.32f * r }, { 0.5f * r, c4 - 0.2f * r }, { 0.0f, c4 - 0.2f * r } }));
            Of{ node, Finish::steel }.Add(Disc(0.58f * r, c4 - 0.2f * r, c4, 0.03f * r));
        }

        NodeParts BuildNode(const arm::ArmGeometry& geometry, std::size_t node)
        {
            const auto& p = geometry.opw;
            const auto r = std::clamp(0.15f * std::max(p.c2, p.c3), 0.015f, 0.08f);
            const auto pedestal = std::max(p.c1 / 4.0f, 0.5f * r);
            NodeParts parts{};
            switch (node)
            {
                case 0:
                    Base(parts, pedestal, r);
                    break;
                case 1:
                    Turret(parts, p, pedestal, r);
                    break;
                case 2:
                    UpperArm(parts, p.c2, r);
                    break;
                case 3:
                    Forearm(parts, p, r);
                    break;
                case 4:
                    WristRoll(parts, r);
                    break;
                case 5:
                    WristPitch(parts, p.c4, r);
                    break;
                default:
                    Flange(parts, p.c4, r);
                    break;
            }
            return parts;
        }
    }

    std::vector<GltfPrimitive> SampleNodeMeshes(const arm::ArmGeometry& geometry, std::size_t node)
    {
        auto parts = BuildNode(geometry, node);
        std::vector<GltfPrimitive> primitives;
        for (std::size_t finish = 0; finish != finishCount; ++finish)
        {
            auto& part = parts[finish];
            if (part.faces.empty())
                continue;
            part.Finish();
            primitives.push_back({ MaterialOf(static_cast<Finish>(finish)), ui::stage::Shade(part, creaseAngleDegrees) });
        }
        return primitives;
    }

    arm::ArmProfile SampleProfile()
    {
        auto profile = arm_host::DefaultArmProfile(arm_host::defaultNodes);
        profile.name = "sample meshed arm";
        for (std::size_t node = 0; node != arm::meshCount; ++node)
            profile.meshes[node] = std::string{ arm::meshNames[node] } + ".gltf";
        return profile;
    }

    std::vector<SampleFile> SampleArmFiles()
    {
        const auto profile = SampleProfile();
        std::vector<SampleFile> files;
        for (std::size_t node = 0; node != arm::meshCount; ++node)
            files.push_back({ std::string{ arm::meshNames[node] } + ".gltf", WriteGltf(arm::meshNames[node], SampleNodeMeshes(profile.config.geometry, node)) });

        infra::StdStringOutputStream::WithStorage stream;
        arm::WriteArmProfile(profile, stream);
        files.push_back({ "profile.json", stream.Storage() });
        return files;
    }
}
