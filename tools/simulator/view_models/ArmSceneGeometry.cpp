#include "tools/simulator/view_models/ArmSceneGeometry.hpp"
#include <algorithm>
#include <cmath>

namespace simulator
{
    namespace
    {
        constexpr float centimetres{ 100.0f };

        using Vector = std::array<float, 3>;

        std::array<float, 4> Quaternion(const math::Matrix3<float>& r)
        {
            const auto trace = r.at(0, 0) + r.at(1, 1) + r.at(2, 2);
            if (trace > 0.0f)
            {
                const auto s = 2.0f * std::sqrt(trace + 1.0f);
                return { 0.25f * s, (r.at(2, 1) - r.at(1, 2)) / s, (r.at(0, 2) - r.at(2, 0)) / s, (r.at(1, 0) - r.at(0, 1)) / s };
            }
            if (r.at(0, 0) > r.at(1, 1) && r.at(0, 0) > r.at(2, 2))
            {
                const auto s = 2.0f * std::sqrt(1.0f + r.at(0, 0) - r.at(1, 1) - r.at(2, 2));
                return { (r.at(2, 1) - r.at(1, 2)) / s, 0.25f * s, (r.at(0, 1) + r.at(1, 0)) / s, (r.at(0, 2) + r.at(2, 0)) / s };
            }
            if (r.at(1, 1) > r.at(2, 2))
            {
                const auto s = 2.0f * std::sqrt(1.0f + r.at(1, 1) - r.at(0, 0) - r.at(2, 2));
                return { (r.at(0, 2) - r.at(2, 0)) / s, (r.at(0, 1) + r.at(1, 0)) / s, 0.25f * s, (r.at(1, 2) + r.at(2, 1)) / s };
            }
            const auto s = 2.0f * std::sqrt(1.0f + r.at(2, 2) - r.at(0, 0) - r.at(1, 1));
            return { (r.at(1, 0) - r.at(0, 1)) / s, (r.at(0, 2) + r.at(2, 0)) / s, (r.at(1, 2) + r.at(2, 1)) / s, 0.25f * s };
        }

        std::array<float, 4> FromUpTo(const Vector& direction)
        {
            const auto dot = direction[1];
            if (dot < -0.9999f)
                return { 0.0f, 1.0f, 0.0f, 0.0f };

            const auto w = std::sqrt((1.0f + dot) / 2.0f);
            return { w, direction[2] / (2.0f * w), 0.0f, -direction[0] / (2.0f * w) };
        }

        class PartBuilder
        {
        public:
            explicit PartBuilder(std::vector<ScenePart>& parts, int joint)
                : parts(parts)
                , joint(joint)
            {}

            void Add(PartShape shape, PartMaterial material, const Vector& from, const Vector& to, float width)
            {
                const Vector axis{ to[0] - from[0], to[1] - from[1], to[2] - from[2] };
                const auto length = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
                if (length < 1.0e-6f)
                    return;

                parts.push_back({ joint, shape, material,
                    { (from[0] + to[0]) * centimetres / 2.0f, (from[1] + to[1]) * centimetres / 2.0f, (from[2] + to[2]) * centimetres / 2.0f },
                    FromUpTo({ axis[0] / length, axis[1] / length, axis[2] / length }), { width, length, width } });
            }

            void Housing(const Vector& centre, const Vector& axis, float diameter, float length)
            {
                Add(PartShape::cylinder, PartMaterial::housing, { centre[0] - axis[0] * length / 2.0f, centre[1] - axis[1] * length / 2.0f, centre[2] - axis[2] * length / 2.0f },
                    { centre[0] + axis[0] * length / 2.0f, centre[1] + axis[1] * length / 2.0f, centre[2] + axis[2] * length / 2.0f }, diameter);
            }

        private:
            std::vector<ScenePart>& parts;
            int joint;
        };
    }

    ScenePose ToScene(const arm::ArmModel::Pose& pose)
    {
        const auto& r = pose.rotation;
        const math::Matrix3<float> converted{ { r.at(0, 0), r.at(0, 1), r.at(0, 2) }, { r.at(2, 0), r.at(2, 1), r.at(2, 2) }, { -r.at(1, 0), -r.at(1, 1), -r.at(1, 2) } };
        const auto& t = pose.translation;
        return { { t[0] * centimetres, t[2] * centimetres, -t[1] * centimetres }, Quaternion(converted) };
    }

    SceneNodes SceneNodePoses(const arm::ArmModel& model, const arm::ArmGeometry& geometry, const arm::JointPositions& joints)
    {
        SceneNodes nodes{};
        nodes[0] = ToScene(arm::ToTransform(geometry.base));
        const auto frames = model.LinkPoses(joints);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            nodes[joint + 1] = ToScene(frames[joint]);
        return nodes;
    }

    float PartThickness(const arm::ArmGeometry& geometry)
    {
        return std::clamp(0.15f * std::max(geometry.opw.c2, geometry.opw.c3), 0.015f, 0.08f);
    }

    float ArmExtent(const arm::ArmGeometry& geometry)
    {
        const auto& p = geometry.opw;
        return centimetres * (p.c1 + std::abs(p.a1) + p.c2 + std::sqrt(p.a2 * p.a2 + p.c3 * p.c3) + p.c4);
    }

    SceneParts BuildSceneParts(const arm::ArmGeometry& geometry)
    {
        const auto& p = geometry.opw;
        const auto r = PartThickness(geometry);
        const Vector up{ 0.0f, 0.0f, 1.0f };
        const Vector side{ 0.0f, 1.0f, 0.0f };
        const auto pedestal = p.c1 / 4.0f;
        SceneParts parts{};

        PartBuilder{ parts[0], -1 }.Add(PartShape::cylinder, PartMaterial::cast, {}, { 0.0f, 0.0f, pedestal }, 2.8f * r);

        PartBuilder turret{ parts[1], 0 };
        turret.Housing({ 0.0f, 0.0f, pedestal + 0.15f * r }, up, 2.4f * r, 0.3f * r);
        turret.Add(PartShape::cylinder, PartMaterial::accent, { 0.0f, 0.0f, pedestal }, { 0.0f, 0.0f, p.c1 }, 2.0f * r);
        turret.Add(PartShape::box, PartMaterial::accent, { 0.0f, 0.0f, p.c1 }, { p.a1, p.b, p.c1 }, 2.0f * r);

        PartBuilder upper{ parts[2], 1 };
        upper.Housing({}, side, 2.0f * r, 2.2f * r);
        upper.Add(PartShape::box, PartMaterial::paint, {}, { 0.0f, 0.0f, p.c2 }, 1.2f * r);

        PartBuilder forearm{ parts[3], 2 };
        forearm.Housing({}, side, 1.8f * r, 2.0f * r);
        forearm.Add(PartShape::box, PartMaterial::paint, {}, { p.a2, 0.0f, 0.0f }, 1.1f * r);
        forearm.Add(PartShape::box, PartMaterial::paint, { p.a2, 0.0f, 0.0f }, { p.a2, 0.0f, std::max(p.c3 - 0.9f * r, 0.0f) }, 1.1f * r);

        PartBuilder{ parts[4], 3 }.Add(PartShape::cylinder, PartMaterial::housing, { 0.0f, 0.0f, -0.9f * r }, {}, 1.1f * r);
        PartBuilder{ parts[5], 4 }.Housing({}, side, r, 1.3f * r);

        PartBuilder flange{ parts[6], 5 };
        flange.Add(PartShape::cylinder, PartMaterial::housing, {}, { 0.0f, 0.0f, p.c4 }, 0.6f * r);
        flange.Add(PartShape::cylinder, PartMaterial::rubber, { 0.0f, 0.0f, p.c4 - 0.1f * r }, { 0.0f, 0.0f, p.c4 }, 1.2f * r);
        return parts;
    }
}
