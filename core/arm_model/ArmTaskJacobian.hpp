#ifndef CORE_ARM_MODEL_ARM_TASK_JACOBIAN_HPP
#define CORE_ARM_MODEL_ARM_TASK_JACOBIAN_HPP

#include "core/arm_model/ArmModel.hpp"
#include "robotics/kinematics/JacobianProvider.hpp"

namespace arm
{
    class ArmTaskJacobian
        : public kinematics::JacobianProvider<float, 6, jointCount>
    {
    public:
        explicit ArmTaskJacobian(const ArmModel& model);

        TaskJacobian Jacobian(const JointVector& q) const override;
        TaskVector BiasAcceleration(const JointVector& q, const JointVector& qDot) const override;
        math::SE3Transform<float> ToolPose(const JointVector& q) const override;

    private:
        const ArmModel& model;
    };
}

#endif
