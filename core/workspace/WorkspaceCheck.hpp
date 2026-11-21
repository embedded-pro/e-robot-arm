#pragma once

#include "core/arm_model/ArmModel.hpp"
#include "core/safety/interfaces/PoseGuard.hpp"
#include "core/workspace/WorkspaceConfig.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include "robotics/planning/PrimitiveDistance.hpp"
#include "robotics/planning/SelfCollision.hpp"
#include <array>
#include <optional>

namespace arm
{
    WorkspaceBox DefaultWorkspace(const ArmGeometry& geometry, const WorkspaceConfig& config);

    class WorkspaceCheck
        : public PoseGuard
    {
    public:
        using Capsules = std::array<planning::Capsule<float>, workspaceCapsules>;

        WorkspaceCheck(const ArmModel& model, const ArmGeometry& geometry, const WorkspaceConfig& config);

        std::optional<WorkspaceViolation> Check(const JointPositions& joints) const override;
        const WorkspaceBox& Workspace() const;
        Capsules LinkCapsules(const JointPositions& joints) const;

    private:
        using Frames = kinematics::FrameChain<float, workspaceCapsules>;

        Frames Chain(const JointPositions& joints) const;
        std::optional<WorkspaceViolation> OutsideWorkspace(const Capsules& capsules) const;
        std::optional<WorkspaceViolation> InForbiddenZone(const Capsules& capsules) const;
        std::optional<WorkspaceViolation> SelfColliding(const Frames& frames) const;
        void AllowPairsTouchingByConstruction();

        const ArmModel& model;
        math::SE3Transform<float> baseInverse;
        float flangeOffset;
        WorkspaceConfig config;
        WorkspaceBox workspace;
        planning::SelfCollision<float, workspaceCapsules> selfCollision;
    };
}
