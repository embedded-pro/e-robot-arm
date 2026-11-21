#pragma once

#include "core/safety/interfaces/PoseGuard.hpp"
#include "gmock/gmock.h"

namespace arm
{
    class PoseGuardMock
        : public PoseGuard
    {
    public:
        MOCK_METHOD(std::optional<WorkspaceViolation>, Check, (const JointPositions& joints), (const, override));
    };
}
