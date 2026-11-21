#pragma once

#include "core/arm_model/ArmGeometry.hpp"
#include "core/arm_model/ArmMass.hpp"
#include "core/arm_model/JointFrameMap.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/ChainDynamicsModel.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include <cstdint>
#include <optional>

namespace arm
{

    struct Singularities
    {
        bool wrist{ false };
        bool elbow{ false };
        bool shoulder{ false };

        bool Any() const
        {
            return wrist || elbow || shoulder;
        }

        bool operator==(const Singularities& other) const = default;
    };

    enum class IkFailure : uint8_t
    {
        unreachable,
        outsideLimits
    };

    struct IkResult
    {
        std::optional<JointPositions> joints;
        IkFailure failure{ IkFailure::unreachable };
    };

    using JointInertias = std::array<float, jointCount>;

    class ArmModel
    {
    public:
        using Pose = math::SE3Transform<float>;
        using Jacobian = math::Matrix<float, 6, 6>;
        using MassMatrix = math::SquareMatrix<float, 6>;
        using LinkFrames = std::array<Pose, jointCount>;
        using FrameChain = kinematics::FrameChain<float, jointCount>;

        static constexpr float singularSine{ 0.01f };
        static constexpr float shoulderDistance{ 0.005f };
        static constexpr float standardGravity{ 9.81f };

        ArmModel(const ArmGeometry& geometry, const ArmMass& mass, const JointLimits& limits);

        Pose ToolPose(const JointPositions& joints) const;
        Jacobian ToolJacobian(const JointPositions& joints) const;
        float Manipulability(const JointPositions& joints) const;
        Singularities Near(const JointPositions& joints) const;
        IkResult Inverse(const Pose& target, const JointPositions& current) const;
        LinkFrames LinkPoses(const JointPositions& joints) const;
        JointTorques GravityTorque(const JointPositions& joints) const;
        JointTorques InverseDynamics(const JointPositions& joints, const JointSpeeds& speeds, const JointAccelerations& accelerations) const;
        MassMatrix JointMassMatrix(const JointPositions& joints) const;

        using JointVector = arm::JointVector;

        using DynamicLinks = std::array<dynamics::JointLink<float>, jointCount>;

        const dynamics::EulerLagrangeDynamics<float, jointCount>& Dynamics() const;
        const dynamics::InverseDynamicsModel<float, jointCount>& InverseModel() const;
        const DynamicLinks& Links() const;
        const math::Vector3<float>& Gravity() const;
        JointVector ModelVector(const JointPositions& joints) const;
        FrameChain Frames(const JointVector& angles) const;
        JointVector ModelSpeeds(const JointSpeeds& speeds) const;
        JointVector ModelAccelerations(const JointAccelerations& accelerations) const;
        JointVector ModelTorques(const JointTorques& torques) const;
        JointPositions ToJointPositions(const JointVector& angles) const;
        JointSpeeds ToJointSpeeds(const JointVector& speeds) const;
        JointTorques ToJointTorques(const JointVector& torques) const;

    private:
        math::Vector3<float> WristCentre(const JointVector& theta) const;
        std::optional<JointPositions> WithinLimits(const JointVector& solution, const JointPositions& current) const;

        ArmGeometry geometry;
        JointFrameMap jointMap;
        JointLimits limits;
        kinematics::AnalyticalIkOpw<float> opw;
        Pose tool;
        Pose base;
        Pose toolInverse;
        Pose baseInverse;
        float forearm;
        float forearmAngle;
        DynamicLinks links;
        math::Vector3<float> gravity;
        dynamics::ChainDynamicsModel<float, jointCount> dynamics;
    };
}
