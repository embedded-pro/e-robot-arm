#include "tools/simulator/view_models/DesktopArm.hpp"
#include "ui/stage/Primitives.hpp"
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <numbers>
#include <vector>

namespace simulator
{
    namespace
    {
        using ui::scene::Matrix3;
        using ui::scene::Transform3;
        using ui::stage::Mesh;
        using Vec3 = ui::scene::Vector3;
        using Frame = Transform3;
        using NodeParts = std::array<Mesh, finishCount>;

        constexpr ui::stage::Tessellation round{ 32 };
        constexpr ui::stage::Tessellation small{ 16 };
        constexpr float creaseAngleDegrees{ 35.0f };
        const Vec3 up{ 0.0f, 0.0f, 1.0f };
        const Vec3 side{ 0.0f, 1.0f, 0.0f };
        const Vec3 otherSide{ 0.0f, -1.0f, 0.0f };

        Frame At(const Vec3& origin)
        {
            return Transform3::Translation(origin);
        }

        Frame Along(const Vec3& origin, const Vec3& axis)
        {
            const auto z = ui::scene::Normalized(axis);
            const Vec3 helper = std::abs(z.z) < 0.9f ? Vec3{ 0.0f, 0.0f, 1.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
            const auto x = ui::scene::Normalized(ui::scene::Cross(helper, z));
            return Frame{ Matrix3{ { x, ui::scene::Cross(z, x), z } }, origin };
        }

        Mesh Lathe(std::initializer_list<ui::stage::ProfilePoint> bottomToTop, ui::stage::Tessellation tessellation)
        {
            const std::vector<ui::stage::ProfilePoint> topToBottom(std::rbegin(bottomToTop), std::rend(bottomToTop));
            return ui::stage::MakeLathe(topToBottom, tessellation);
        }

        Mesh Extrude(const ui::stage::Outline& outline, float bottom, float top)
        {
            return ui::stage::MakeExtrusion(outline, bottom, top);
        }

        struct Of
        {
            NodeParts& node;
            ArmFinish finish;

            void Add(const Mesh& part, const Frame& frame = {}) const
            {
                node[static_cast<std::size_t>(finish)].Append(part, frame);
            }
        };

        Mesh Disc(float radius, float bottom, float top, float chamfer)
        {
            return Lathe({ { 0.0f, bottom }, { radius, bottom }, { radius, top - chamfer }, { radius - chamfer, top }, { 0.0f, top } }, round);
        }

        Mesh Hub(float radius, float length, float chamfer)
        {
            const auto h = length / 2.0f;
            return Lathe({ { 0.0f, -h }, { radius - chamfer, -h }, { radius, -h + chamfer }, { radius, h - chamfer }, { radius - chamfer, h }, { 0.0f, h } }, round);
        }

        Mesh Gearbox(float radius, float length)
        {
            const auto r = radius;
            const auto l = length;
            return Lathe({ { 0.0f, 0.0f }, { 0.85f * r, 0.0f }, { r, 0.12f * l }, { r, 0.62f * l }, { 0.9f * r, 0.62f * l }, { 0.9f * r, 0.74f * l },
                             { r, 0.74f * l }, { r, 0.9f * l }, { 0.8f * r, l }, { 0.0f, l } },
                round);
        }

        Mesh Rod(float radius, float length)
        {
            return Lathe({ { 0.0f, -length / 2.0f }, { radius, -length / 2.0f }, { radius, length / 2.0f }, { 0.0f, length / 2.0f } }, small);
        }

        void Motor(NodeParts& node, const Frame& frame, float size, float length)
        {
            const auto plate = 0.08f * size;
            Of{ node, ArmFinish::steel }.Add(Extrude(ui::stage::RoundedRectangle(size, size, 0.1f * size, 2), 0.0f, plate), frame);
            Of{ node, ArmFinish::motor }.Add(Extrude(ui::stage::RoundedRectangle(0.94f * size, 0.94f * size, 0.18f * size, 3), plate, length - 0.06f * size), frame);
            Of{ node, ArmFinish::steel }.Add(Extrude(ui::stage::RoundedRectangle(size, size, 0.1f * size, 2), length - 0.06f * size, length), frame);
            Of{ node, ArmFinish::rubber }.Add(Disc(0.18f * size, length, length + 0.05f * size, 0.02f * size), frame);
        }

        void Bolts(NodeParts& node, const Frame& frame, std::size_t count, float circle, float size)
        {
            const auto head = Extrude(ui::stage::RegularPolygon(6, size), 0.0f, 0.6f * size);
            for (std::size_t bolt = 0; bolt != count; ++bolt)
            {
                const auto angle = 2.0f * std::numbers::pi_v<float> * (static_cast<float>(bolt) + 0.5f) / static_cast<float>(count);
                Of{ node, ArmFinish::steel }.Add(head, Frame{ frame.rotation, frame.Apply({ circle * std::cos(angle), circle * std::sin(angle), 0.0f }) });
            }
        }

        Frame Sideways(const Vec3& origin, const Vec3& length, const Vec3& thickness)
        {
            return Frame{ Matrix3{ { ui::scene::Cross(length, thickness), length, thickness } }, origin };
        }

        void Plate(NodeParts& node, ArmFinish finish, const Vec3& start, const Vec3& direction, float length, float width, float thickness)
        {
            Of{ node, finish }.Add(Extrude(ui::stage::Stadium(length, width, 16), -thickness / 2.0f, thickness / 2.0f), Sideways(start, direction, side));
        }

        void Base(NodeParts& node, const arm::ArmGeometry& geometry, float r)
        {
            const auto pedestal = std::max(geometry.opw.c1 / 4.0f, 0.5f * r);
            Of{ node, ArmFinish::anodised }.Add(Disc(2.3f * r, 0.0f, 0.22f * r, 0.05f * r));
            Bolts(node, At({ 0.0f, 0.0f, 0.22f * r }), 6, 1.95f * r, 0.13f * r);
            Of{ node, ArmFinish::aluminium }
                .Add(Lathe({ { 0.0f, 0.22f * r }, { 1.45f * r, 0.22f * r }, { 1.5f * r, 0.3f * r }, { 1.5f * r, pedestal - 0.08f * r }, { 1.4f * r, pedestal }, { 0.0f, pedestal } }, round));
        }

        void Turret(NodeParts& node, const arm::ArmGeometry& geometry, float r)
        {
            const auto& p = geometry.opw;
            const auto pedestal = std::max(p.c1 / 4.0f, 0.5f * r);
            const auto deck = pedestal + 0.18f * r;
            const auto yoke = std::max(p.c1 - 1.6f * r, deck + 0.3f * r);
            Of{ node, ArmFinish::accent }.Add(Disc(1.55f * r, pedestal, deck, 0.05f * r));
            Of{ node, ArmFinish::aluminium }.Add(Extrude(ui::stage::RoundedRectangle(1.7f * r, 1.5f * r, 0.3f * r, 4), deck, yoke));
            Of{ node, ArmFinish::aluminium }.Add(Extrude(ui::stage::RoundedRectangle(std::abs(p.a1) + 2.0f * r, 2.0f * r, 0.25f * r, 4), yoke - 0.3f * r, yoke), At({ p.a1 / 2.0f, p.b, 0.0f }));
            for (const auto offset : { -0.9f * r, 0.9f * r })
                Plate(node, ArmFinish::aluminium, { p.a1, p.b + offset, yoke - 0.3f * r }, up, p.c1 - yoke + 0.3f * r, 2.0f * r, 0.16f * r);
            Of{ node, ArmFinish::anodised }.Add(Gearbox(0.8f * r, 0.55f * r), Along({ p.a1, p.b - 0.98f * r, p.c1 }, otherSide));
            Motor(node, Along({ p.a1, p.b - 1.53f * r, p.c1 }, otherSide), 1.1f * r, 1.1f * r);
        }

        void UpperArm(NodeParts& node, const arm::ArmGeometry& geometry, float r)
        {
            const auto c2 = geometry.opw.c2;
            Of{ node, ArmFinish::accent }.Add(Hub(0.75f * r, 1.5f * r, 0.08f * r), Along({}, side));
            for (const auto offset : { -0.42f * r, 0.42f * r })
                Plate(node, ArmFinish::aluminium, { 0.0f, offset, 0.0f }, up, c2, 1.3f * r, 0.14f * r);
            for (const auto fraction : { 0.33f, 0.66f })
                Of{ node, ArmFinish::steel }.Add(Rod(0.12f * r, 0.84f * r), Along({ 0.0f, 0.0f, fraction * c2 }, side));
            Of{ node, ArmFinish::anodised }.Add(Gearbox(0.6f * r, 0.45f * r), Along({ 0.0f, -0.5f * r, c2 }, otherSide));
            Motor(node, Along({ 0.0f, -0.95f * r, c2 }, otherSide), 0.95f * r, 0.9f * r);
        }

        void Forearm(NodeParts& node, const arm::ArmGeometry& geometry, float r)
        {
            const auto& p = geometry.opw;
            const Vec3 reach{ p.a2, 0.0f, p.c3 };
            const auto length = std::sqrt(p.a2 * p.a2 + p.c3 * p.c3);
            const Vec3 along{ p.a2 / length, 0.0f, p.c3 / length };
            const Vec3 front{ along.z, 0.0f, -along.x };
            Of{ node, ArmFinish::accent }.Add(Hub(0.6f * r, 0.7f * r, 0.06f * r), Along({}, side));
            Of{ node, ArmFinish::aluminium }.Add(Extrude(ui::stage::RoundedRectangle(1.0f * r, 0.9f * r, 0.25f * r, 4), 0.0f, std::max(length - 0.95f * r, 0.1f * r)), Along({}, reach));
            const auto start = 0.12f * length;
            Motor(node, Along({ along.x * start + front.x * 0.84f * r, 0.0f, along.z * start + front.z * 0.84f * r }, reach), 0.8f * r, 0.32f * length);
        }

        void WristRoll(NodeParts& node, float r)
        {
            Of{ node, ArmFinish::anodised }.Add(Lathe({ { 0.0f, -0.95f * r }, { 0.5f * r, -0.95f * r }, { 0.55f * r, -0.9f * r }, { 0.55f * r, -0.35f * r }, { 0.0f, -0.35f * r } }, round));
            Of{ node, ArmFinish::accent }.Add(Disc(0.6f * r, -0.35f * r, -0.25f * r, 0.02f * r));
            for (const auto offset : { -0.48f * r, 0.48f * r })
                Plate(node, ArmFinish::aluminium, { 0.0f, offset, -0.3f * r }, up, 0.3f * r, 1.0f * r, 0.12f * r);
            Of{ node, ArmFinish::anodised }.Add(Gearbox(0.42f * r, 0.35f * r), Along({ 0.0f, 0.55f * r, 0.0f }, side));
            Motor(node, Along({ 0.0f, 0.9f * r, 0.0f }, side), 0.75f * r, 0.7f * r);
        }

        void WristPitch(NodeParts& node, const arm::ArmGeometry& geometry, float r)
        {
            Of{ node, ArmFinish::accent }.Add(Hub(0.42f * r, 0.84f * r, 0.05f * r), Along({}, side));
            const auto top = std::max(geometry.opw.c4 - 0.32f * r, 0.45f * r);
            Of{ node, ArmFinish::aluminium }.Add(Extrude(ui::stage::RoundedRectangle(0.75f * r, 0.75f * r, 0.15f * r, 3), 0.3f * r, top));
        }

        void Flange(NodeParts& node, const arm::ArmGeometry& geometry, float r)
        {
            const auto c4 = geometry.opw.c4;
            Of{ node, ArmFinish::anodised }.Add(Lathe({ { 0.0f, c4 - 0.32f * r }, { 0.5f * r, c4 - 0.32f * r }, { 0.5f * r, c4 - 0.2f * r }, { 0.0f, c4 - 0.2f * r } }, round));
            Of{ node, ArmFinish::steel }.Add(Disc(0.55f * r, c4 - 0.2f * r, c4, 0.04f * r));
            Bolts(node, At({ 0.0f, 0.0f, c4 }), 4, 0.36f * r, 0.06f * r);
        }
    }

    float PartThickness(const arm::ArmGeometry& geometry)
    {
        return std::clamp(0.15f * std::max(geometry.opw.c2, geometry.opw.c3), 0.015f, 0.08f);
    }

    ArmMeshes BuildDesktopArm(const arm::ArmGeometry& geometry)
    {
        const auto r = PartThickness(geometry);
        std::array<NodeParts, armNodeCount> parts{};
        Base(parts[0], geometry, r);
        Turret(parts[1], geometry, r);
        UpperArm(parts[2], geometry, r);
        Forearm(parts[3], geometry, r);
        WristRoll(parts[4], r);
        WristPitch(parts[5], geometry, r);
        Flange(parts[6], geometry, r);

        ArmMeshes arm{};
        for (std::size_t node = 0; node != armNodeCount; ++node)
            for (std::size_t finish = 0; finish != finishCount; ++finish)
            {
                auto& part = parts[node][finish];
                if (part.faces.empty())
                    continue;
                part.Finish();
                arm[node][finish] = ui::stage::Shade(part, creaseAngleDegrees);
            }
        return arm;
    }
}
