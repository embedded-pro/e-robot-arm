#pragma once

#include "core/joint_bus/interfaces/JointBus.hpp"
#include <cstdint>
#include <optional>

namespace arm
{
    enum class WorkspaceRule : uint8_t
    {
        workspace,
        forbiddenZone,
        selfCollision
    };

    struct WorkspaceViolation
    {
        WorkspaceRule rule{ WorkspaceRule::workspace };
        uint8_t link{ 0 };
        uint8_t other{ 0 };
        float distance{ 0.0f };

        bool operator==(const WorkspaceViolation& other) const = default;
    };

    class PoseGuard
    {
    public:
        PoseGuard() = default;
        PoseGuard(const PoseGuard& other) = delete;
        PoseGuard& operator=(const PoseGuard& other) = delete;

        virtual std::optional<WorkspaceViolation> Check(const JointPositions& joints) const = 0;

    protected:
        ~PoseGuard() = default;
    };
}
