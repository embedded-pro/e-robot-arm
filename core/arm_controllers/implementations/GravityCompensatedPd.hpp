#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_GRAVITY_COMPENSATED_PD_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_GRAVITY_COMPENSATED_PD_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_model/ArmModel.hpp"
#include "robotics/controllers/manipulator/PdGravityCompensation.hpp"
#include <array>

namespace arm
{
    struct PdGain
    {
        float kp{ 0.0f };
        float kd{ 0.0f };
    };

    using JointPdGains = std::array<PdGain, jointCount>;

    inline JointPdGains ReferenceGravityGains()
    {
        JointPdGains gains{};
        gains.fill(PdGain{ 20.0f, 0.5f });
        return gains;
    }

    class GravityCompensatedPd
        : public ArmController
    {
    public:
        GravityCompensatedPd(const ArmModel& model, const JointPdGains& gains);

        bool BumplessTransfer() const override;
        can::FocMotorMode Mode() const override;
        void Reset(const MeasuredJoints& measured) override;
        JointSetpoints Step(const MeasuredJoints& measured, const JointPositions& reference) override;

    private:
        using Law = controllers::PdGravityCompensation<float, jointCount>;

        const ArmModel& model;
        Law law;
    };
}

#endif
