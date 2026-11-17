#ifndef CORE_COLLISION_COLLISION_OBSERVER_HPP
#define CORE_COLLISION_COLLISION_OBSERVER_HPP

#include "core/arm_model/ArmModel.hpp"
#include "infra/timer/Timer.hpp"
#include "robotics/dynamics/ChainDynamicsModel.hpp"
#include "robotics/dynamics/CoriolisMatrix.hpp"
#include "robotics/dynamics/MomentumObserver.hpp"

namespace arm
{
    class CollisionObserver
    {
    public:
        CollisionObserver(const ArmModel& model, const JointInertias& rotors, float gain);

        void Reset(const JointPositions& positions, const JointSpeeds& speeds);
        JointTorques Update(const JointPositions& positions, const JointSpeeds& speeds, const JointTorques& commanded, infra::Duration period);

    private:
        using Observer = dynamics::MomentumObserver<float, jointCount>;

        const ArmModel& model;
        ArmModel::DynamicLinks links;
        dynamics::ChainDynamicsModel<float, jointCount> dynamics;
        dynamics::CoriolisMatrix<float, jointCount> coriolis;
        Observer observer;
    };
}

#endif
