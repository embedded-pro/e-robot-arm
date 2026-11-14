#ifndef CORE_ARM_MODEL_ARM_MODEL_HPP
#define CORE_ARM_MODEL_ARM_MODEL_HPP

#include "core/arm_model/ArmGeometry.hpp"
#include "core/arm_model/ArmMass.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/ChainDynamicsModel.hpp"
#include <cstdint>
#include <optional>

namespace arm
{
    using JointAccelerations = std::array<foc::RadiansPerSecondSquared, jointCount>;
    using JointTorques = std::array<foc::NewtonMeter, jointCount>;

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

    class ArmModel
    {
    public:
        using Pose = math::SE3Transform<float>;
        using Jacobian = math::Matrix<float, 6, 6>;
        using MassMatrix = math::SquareMatrix<float, 6>;
        using LinkFrames = std::array<Pose, jointCount>;

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

    private:
        using JointVector = math::Vector<float, 6>;

        struct ModelAngles
        {
            std::array<float, 6> theta{};
        };

        ModelAngles Angles(const JointPositions& joints) const;
        math::Vector3<float> WristCentre(const ModelAngles& angles) const;
        std::optional<JointPositions> WithinLimits(const JointVector& solution, const JointPositions& current) const;
        JointVector ModelVector(const JointPositions& joints) const;
        JointTorques ToJointTorques(const JointVector& torques) const;

        ArmGeometry geometry;
        JointLimits limits;
        kinematics::AnalyticalIkOpw<float> opw;
        Pose tool;
        Pose base;
        Pose toolInverse;
        Pose baseInverse;
        float forearm;
        float forearmAngle;
        dynamics::ChainDynamicsModel<float, jointCount> dynamics;
    };
}

#endif
