#ifndef CORE_COLLISION_COLLISION_OBSERVER_HPP
#define CORE_COLLISION_COLLISION_OBSERVER_HPP

#include "core/arm_model/DrivenArm.hpp"
#include "infra/timer/Timer.hpp"
#include "robotics/dynamics/CoriolisMatrix.hpp"
#include "robotics/dynamics/MomentumObserver.hpp"

namespace arm
{
    class CollisionObserver
    {
    public:
        CollisionObserver(const DrivenArm& drivenArm, float gain);

        void Reset(const JointPositions& positions, const JointSpeeds& speeds);
        JointTorques Update(const JointPositions& positions, const JointSpeeds& speeds, const JointTorques& commanded, infra::Duration period);

    private:
        using Observer = dynamics::MomentumObserver<float, jointCount>;

        const ArmModel& model;
        dynamics::CoriolisMatrix<float, jointCount> coriolis;
        Observer observer;
    };
}

#endif
