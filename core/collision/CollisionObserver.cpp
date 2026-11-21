#include "core/collision/CollisionObserver.hpp"
#include <chrono>

namespace arm
{
    namespace
    {
        ArmModel::JointVector Uniform(float gain)
        {
            ArmModel::JointVector gains{};
            for (std::size_t joint = 0; joint != jointCount; ++joint)
                gains.at(joint, 0) = gain;
            return gains;
        }
    }

    CollisionObserver::CollisionObserver(const DrivenArm& drivenArm, float gain)
        : model(drivenArm.Model())
        , observer(drivenArm.LinkDynamics(), coriolis, drivenArm.Links(), Uniform(gain))
    {}

    void CollisionObserver::Reset(const JointPositions& positions, const JointSpeeds& speeds)
    {
        observer.Reset(model.ModelVector(positions), model.ModelSpeeds(speeds));
    }

    JointTorques CollisionObserver::Update(const JointPositions& positions, const JointSpeeds& speeds, const JointTorques& commanded, infra::Duration period)
    {
        const auto seconds = std::chrono::duration<float>(period).count();
        return model.ToJointTorques(observer.Update(model.ModelVector(positions), model.ModelSpeeds(speeds), model.ModelTorques(commanded), seconds));
    }
}
