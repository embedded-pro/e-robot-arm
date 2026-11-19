#include "tools/simulator/view_models/ArmSceneGeometry.hpp"
#include <algorithm>
#include <cmath>

namespace simulator
{
    namespace
    {
        constexpr float centimetres{ 100.0f };

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
    }

    ScenePose ToScene(const arm::ArmModel::Pose& pose)
    {
        const auto& r = pose.rotation;
        const math::Matrix3<float> converted{ { r.at(0, 0), r.at(0, 1), r.at(0, 2) }, { r.at(2, 0), r.at(2, 1), r.at(2, 2) }, { -r.at(1, 0), -r.at(1, 1), -r.at(1, 2) } };
        const auto& t = pose.translation;
        return { { t[0] * centimetres, t[2] * centimetres, -t[1] * centimetres }, Quaternion(converted) };
    }

    SceneBox ToSceneBox(const arm::WorkspaceBox& box, SceneBoxKind kind)
    {
        std::array<float, 3> centre{};
        std::array<float, 3> size{};
        for (std::size_t axis = 0; axis != 3; ++axis)
        {
            centre[axis] = 0.5f * (box.min[axis] + box.max[axis]) * centimetres;
            size[axis] = std::abs(box.max[axis] - box.min[axis]) * centimetres;
        }
        return { kind, { centre[0], centre[2], -centre[1] }, { size[0], size[2], size[1] }, false };
    }

    std::array<std::array<float, 3>, 16> BoxOutline(const SceneBox& box)
    {
        constexpr std::array<std::array<int, 3>, 16> corners{ { { -1, -1, -1 }, { 1, -1, -1 }, { 1, -1, 1 }, { -1, -1, 1 }, { -1, -1, -1 }, { -1, 1, -1 },
            { 1, 1, -1 }, { 1, -1, -1 }, { 1, 1, -1 }, { 1, 1, 1 }, { 1, -1, 1 }, { 1, 1, 1 }, { -1, 1, 1 }, { -1, -1, 1 }, { -1, 1, 1 }, { -1, 1, -1 } } };
        std::array<std::array<float, 3>, 16> outline{};
        for (std::size_t point = 0; point != corners.size(); ++point)
            for (std::size_t axis = 0; axis != 3; ++axis)
                outline[point][axis] = box.centre[axis] + 0.5f * static_cast<float>(corners[point][axis]) * box.size[axis];
        return outline;
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

    float ArmExtent(const arm::ArmGeometry& geometry)
    {
        const auto& p = geometry.opw;
        return centimetres * (p.c1 + std::abs(p.a1) + p.c2 + std::sqrt(p.a2 * p.a2 + p.c3 * p.c3) + p.c4);
    }

}
