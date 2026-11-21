#include "core/workspace/WorkspaceCheck.hpp"
#include "numerical/math/Math.hpp"
#include "robotics/planning/ConvexCollision.hpp"
#include <algorithm>

namespace arm
{
    namespace
    {
        using Collision = planning::ConvexCollision<float>;
        using Shape = Collision::Shape;

        float OutsideBy(const math::Vector3<float>& point, const WorkspaceBox& box, float radius)
        {
            float outside{ 0.0f };
            for (std::size_t axis = 0; axis != 3; ++axis)
                outside = std::max({ outside, box.min[axis] + radius - point[axis], point[axis] - (box.max[axis] - radius) });
            return outside;
        }

        Shape ZoneShape(const WorkspaceBox& zone)
        {
            const math::Vector3<float> centre{ (zone.min + zone.max) * 0.5f };
            const math::Vector3<float> halfExtents{ (zone.max - zone.min) * 0.5f };
            return Shape::Box(math::SE3Transform<float>{ math::SquareMatrix<float, 3>::Identity(), centre }, halfExtents);
        }

        WorkspaceViolation Violation(WorkspaceRule rule, std::size_t link, std::size_t other, float distance)
        {
            return { rule, static_cast<uint8_t>(link), static_cast<uint8_t>(other), distance };
        }
    }

    WorkspaceBox DefaultWorkspace(const ArmGeometry& geometry, const WorkspaceConfig& config)
    {
        const auto& p = geometry.opw;
        const auto& tool = geometry.tool;
        const auto reach = math::Abs(p.a1) + math::Abs(p.b) + math::Abs(p.c1) + p.c2 + math::Sqrt(p.a2 * p.a2 + p.c3 * p.c3) + p.c4 +
                           math::VectorNorm(math::Vector3<float>{ tool.x, tool.y, tool.z }) + *std::ranges::max_element(config.linkRadii);
        return { math::Vector3<float>{ -reach, -reach, 0.0f }, math::Vector3<float>{ reach, reach, reach } };
    }

    WorkspaceCheck::WorkspaceCheck(const ArmModel& model, const ArmGeometry& geometry, const WorkspaceConfig& config)
        : model(model)
        , baseInverse(ToTransform(geometry.base).Inverse())
        , flangeOffset(geometry.opw.c4)
        , config(config)
        , workspace(config.workspace.value_or(DefaultWorkspace(geometry, config)))
        , selfCollision(config.linkRadii)
    {
        AllowPairsTouchingByConstruction();
        for (std::size_t pair = 0; pair != config.allowedLinkPairCount; ++pair)
            selfCollision.Allow(config.allowedLinkPairs[pair][0], config.allowedLinkPairs[pair][1]);
    }

    std::optional<WorkspaceViolation> WorkspaceCheck::Check(const JointPositions& joints) const
    {
        const auto frames = Chain(joints);
        Capsules capsules{};
        for (std::size_t link = 0; link != workspaceCapsules; ++link)
            capsules[link] = selfCollision.LinkCapsule(frames, link);

        if (const auto outside = OutsideWorkspace(capsules); outside.has_value())
            return outside;
        if (const auto zone = InForbiddenZone(capsules); zone.has_value())
            return zone;
        return SelfColliding(frames);
    }

    const WorkspaceBox& WorkspaceCheck::Workspace() const
    {
        return workspace;
    }

    WorkspaceCheck::Capsules WorkspaceCheck::LinkCapsules(const JointPositions& joints) const
    {
        const auto frames = Chain(joints);
        Capsules capsules{};
        for (std::size_t link = 0; link != workspaceCapsules; ++link)
            capsules[link] = selfCollision.LinkCapsule(frames, link);
        return capsules;
    }

    WorkspaceCheck::Frames WorkspaceCheck::Chain(const JointPositions& joints) const
    {
        const auto links = model.LinkPoses(joints);
        Frames frames{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            frames.jointOrigins[joint] = baseInverse.Apply(links[joint].translation);
        frames.jointOrigins[jointCount] = baseInverse.Apply(links.back().Apply(math::Vector3<float>{ 0.0f, 0.0f, flangeOffset }));
        frames.tool = baseInverse * model.ToolPose(joints);
        return frames;
    }

    std::optional<WorkspaceViolation> WorkspaceCheck::OutsideWorkspace(const Capsules& capsules) const
    {
        for (std::size_t link = 1; link != workspaceCapsules; ++link)
        {
            const auto& capsule = capsules[link];
            const auto outside = std::max(OutsideBy(capsule.axis.start, workspace, capsule.radius), OutsideBy(capsule.axis.end, workspace, capsule.radius));
            if (outside > 0.0f)
                return Violation(WorkspaceRule::workspace, link, link, outside);
        }
        return std::nullopt;
    }

    std::optional<WorkspaceViolation> WorkspaceCheck::InForbiddenZone(const Capsules& capsules) const
    {
        for (std::size_t zone = 0; zone != config.forbiddenZoneCount; ++zone)
        {
            const auto box = ZoneShape(config.forbiddenZones[zone]);
            for (std::size_t link = 0; link != workspaceCapsules; ++link)
            {
                const auto& capsule = capsules[link];
                const auto proximity = Collision::Query(Shape::Capsule(capsule.axis.start, capsule.axis.end, capsule.radius), box);
                if (proximity.distance <= 0.0f)
                    return Violation(WorkspaceRule::forbiddenZone, link, zone, -proximity.distance);
            }
        }
        return std::nullopt;
    }

    std::optional<WorkspaceViolation> WorkspaceCheck::SelfColliding(const Frames& frames) const
    {
        const auto result = selfCollision.Check(frames, config.selfCollisionMargin);
        if (!result.colliding)
            return std::nullopt;
        return Violation(WorkspaceRule::selfCollision, result.pair[0], result.pair[1], result.minimumDistance);
    }

    void WorkspaceCheck::AllowPairsTouchingByConstruction()
    {
        const auto reference = Chain(JointPositions{});
        for (std::size_t link = 0; link != workspaceCapsules; ++link)
            for (std::size_t other = link + 2; other < workspaceCapsules; ++other)
            {
                const auto distance = planning::PrimitiveDistance<float>::CapsuleCapsule(selfCollision.LinkCapsule(reference, link), selfCollision.LinkCapsule(reference, other)).distance;
                if (distance < config.selfCollisionMargin)
                    selfCollision.Allow(link, other);
            }
    }
}
