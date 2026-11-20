#include "core/arm_model/ArmTaskJacobian.hpp"
#include "robotics/kinematics/GeometricJacobian.hpp"

namespace arm
{
    ArmTaskJacobian::ArmTaskJacobian(const ArmModel& model)
        : model(model)
    {}

    ArmTaskJacobian::TaskJacobian ArmTaskJacobian::Jacobian(const JointVector& q) const
    {
        return kinematics::GeometricJacobian<float, jointCount>::Compute(model.Frames(q));
    }

    ArmTaskJacobian::TaskVector ArmTaskJacobian::BiasAcceleration(const JointVector& q, const JointVector& qDot) const
    {
        return kinematics::GeometricJacobian<float, jointCount>::BiasAcceleration(model.Frames(q), qDot);
    }

    math::SE3Transform<float> ArmTaskJacobian::ToolPose(const JointVector& q) const
    {
        return model.Frames(q).tool;
    }
}
