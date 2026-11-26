#pragma once

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/DrivenArm.hpp"
#include <cstdint>

namespace arm
{
    enum class PassiveLaw : uint8_t
    {
        free,
        gravityFloat
    };

    struct JointLimitWall
    {
        foc::Radians band{ 0.1f };
        float bandwidth{ 20.0f };
    };

    class PassiveController
        : public ArmController
    {
    public:
        PassiveController(const DrivenArm& drivenArm, const JointLimits& limits, PassiveLaw law, const JointLimitWall& wall = {});

        bool BumplessTransfer() const override;
        void Reset(const MeasuredJoints& measured, const JointReference& held) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointReference& reference) override;

    private:
        float WallTorque(std::size_t joint, const MeasuredJoints& measured, float inertia) const;

        const DrivenArm& drivenArm;
        JointLimits limits;
        PassiveLaw law;
        JointLimitWall wall;
    };
}
