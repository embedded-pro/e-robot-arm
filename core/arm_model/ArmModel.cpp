#include "core/arm_model/ArmModel.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include "numerical/solvers/LuDecomposition.hpp"
#include "robotics/kinematics/GeometricJacobian.hpp"

namespace arm
{
    namespace
    {
        math::Vector3<float> Column(const math::Matrix3<float>& rotation, std::size_t column)
        {
            return math::Vector3<float>{ rotation.at(0, column), rotation.at(1, column), rotation.at(2, column) };
        }

        math::Vector3<float> Point(const std::array<float, 3>& point)
        {
            return math::Vector3<float>{ point[0], point[1], point[2] };
        }

        math::Matrix3<float> Tensor(const std::array<float, 6>& inertia)
        {
            return math::Matrix3<float>{ { inertia[0], inertia[3], inertia[4] }, { inertia[3], inertia[1], inertia[5] }, { inertia[4], inertia[5], inertia[2] } };
        }

        dynamics::JointLink<float> WithPayload(const dynamics::JointLink<float>& link, const MassProperties& payload, float flange)
        {
            return dynamics::AttachRigidBody(link, payload.mass, Point(payload.centre) + math::Vector3<float>{ 0.0f, 0.0f, flange }, Tensor(payload.inertia));
        }

        kinematics::JointLimitArray<float, jointCount> IkLimits(const JointLimits& limits)
        {
            kinematics::JointLimitArray<float, jointCount> ikLimits{};
            for (std::size_t joint = 0; joint != jointCount; ++joint)
            {
                ikLimits[joint].positionMin = limits[joint].lower.Value();
                ikLimits[joint].positionMax = limits[joint].upper.Value();
            }
            return ikLimits;
        }

        struct ChainFrame
        {
            math::Vector3<float> axis;
            math::Vector3<float> origin;
        };

        std::array<ChainFrame, jointCount> Chain(const kinematics::OpwParameters<float>& p)
        {
            const math::Vector3<float> up{ 0.0f, 0.0f, 1.0f };
            const math::Vector3<float> side{ 0.0f, 1.0f, 0.0f };
            return { ChainFrame{ up, math::Vector3<float>{} }, ChainFrame{ side, math::Vector3<float>{ p.a1, p.b, p.c1 } },
                ChainFrame{ side, math::Vector3<float>{ 0.0f, 0.0f, p.c2 } }, ChainFrame{ up, math::Vector3<float>{ p.a2, 0.0f, p.c3 } },
                ChainFrame{ side, math::Vector3<float>{} }, ChainFrame{ up, math::Vector3<float>{} } };
        }

        std::array<dynamics::JointLink<float>, jointCount> BuildLinks(const ArmGeometry& geometry, const ArmMass& mass)
        {
            const auto& p = geometry.opw;
            const auto chain = Chain(p);
            std::array<dynamics::JointLink<float>, jointCount> links{};
            for (std::size_t joint = 0; joint != jointCount; ++joint)
            {
                const auto& properties = mass.links[joint];
                links[joint] = dynamics::JointLink<float>{ properties.mass, Tensor(properties.inertia), chain[joint].axis, chain[joint].origin, Point(properties.centre) };
            }

            links.back() = WithPayload(links.back(), mass.payload, p.c4);
            return links;
        }

        math::Vector3<float> GravityInBase(const math::SE3Transform<float>& base)
        {
            return base.rotation.Transpose() * math::Vector3<float>{ 0.0f, 0.0f, -ArmModel::standardGravity };
        }
    }

    math::SE3Transform<float> ToTransform(const Placement& placement)
    {
        return { math::RotationZ(placement.yaw) * math::RotationY(placement.pitch) * math::RotationX(placement.roll), math::Vector3<float>{ placement.x, placement.y, placement.z } };
    }

    Placement ToPlacement(const math::SE3Transform<float>& transform)
    {
        const auto& r = transform.rotation;
        const auto horizontal = math::Sqrt(r.at(0, 0) * r.at(0, 0) + r.at(1, 0) * r.at(1, 0));
        Placement placement{ transform.translation[0], transform.translation[1], transform.translation[2], 0.0f, math::Atan2(-r.at(2, 0), horizontal), 0.0f };
        if (horizontal > 1.0e-6f)
        {
            placement.roll = math::Atan2(r.at(2, 1), r.at(2, 2));
            placement.yaw = math::Atan2(r.at(1, 0), r.at(0, 0));
        }
        else
            placement.roll = math::Atan2(-r.at(1, 2), r.at(1, 1));
        return placement;
    }

    ArmModel::ArmModel(const ArmGeometry& geometry, const ArmMass& mass, const JointLimits& limits)
        : geometry(geometry)
        , jointMap(geometry.opw.signs, geometry.opw.offsets)
        , ikLimits(IkLimits(limits))
        , opw(geometry.opw)
        , tool(ToTransform(geometry.tool))
        , base(ToTransform(geometry.base))
        , toolInverse(tool.Inverse())
        , baseInverse(base.Inverse())
        , forearm(math::Sqrt(geometry.opw.a2 * geometry.opw.a2 + geometry.opw.c3 * geometry.opw.c3))
        , forearmAngle(math::Atan2(geometry.opw.a2, geometry.opw.c3))
        , links(BuildLinks(geometry, mass))
        , gravity(GravityInBase(base))
        , dynamics(links, gravity)
    {}

    ArmModel::Pose ArmModel::ToolPose(const JointPositions& joints) const
    {
        return base * opw.Forward(ToVector(joints)) * tool;
    }

    ArmModel::Jacobian ArmModel::ToolJacobian(const JointPositions& joints) const
    {
        return jointMap.FlipColumns(kinematics::GeometricJacobian<float, jointCount>::Compute(Frames(ModelVector(joints))));
    }

    ArmModel::FrameChain ArmModel::Frames(const JointVector& theta) const
    {
        const auto& p = geometry.opw;

        const auto shoulder = math::RotationZ(theta[0]);
        const auto upper = shoulder * math::RotationY(theta[1] + theta[2]);
        const auto wrist4 = upper * math::RotationZ(theta[3]);
        const auto wrist5 = wrist4 * math::RotationY(theta[4]);
        const auto centre = WristCentre(theta);

        const std::array<math::Vector3<float>, jointCount> axes{ math::Vector3<float>{ 0.0f, 0.0f, 1.0f }, Column(shoulder, 1), Column(shoulder, 1), Column(upper, 2),
            Column(wrist4, 1), Column(wrist5, 2) };
        const std::array<math::Vector3<float>, jointCount> points{ math::Vector3<float>{},
            shoulder * math::Vector3<float>{ p.a1, 0.0f, p.c1 },
            shoulder * math::Vector3<float>{ p.a1 + p.c2 * math::Sin(theta[1]), 0.0f, p.c1 + p.c2 * math::Cos(theta[1]) }, centre, centre, centre };

        FrameChain frames{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            frames.jointAxes[joint] = base.Rotate(axes[joint]);
            frames.jointOrigins[joint] = base.Apply(points[joint]);
            frames.jointTypes[joint] = dynamics::JointType::Revolute;
        }
        frames.tool = ToolPose(ToJointPositions(theta));
        return frames;
    }

    float ArmModel::Manipulability(const JointPositions& joints) const
    {
        solvers::LuDecomposition<float, 6> lu;
        if (!lu.Decompose(ToolJacobian(joints)))
            return 0.0f;
        return math::Abs(lu.Determinant());
    }

    Singularities ArmModel::Near(const JointPositions& joints) const
    {
        const auto theta = ModelVector(joints);
        const auto centre = WristCentre(theta);
        const auto radial = math::Sqrt(centre[0] * centre[0] + centre[1] * centre[1]);
        return { math::Abs(math::Sin(theta[4])) < singularSine, math::Abs(math::Sin(theta[2] + forearmAngle)) < singularSine,
            math::Abs(radial - math::Abs(geometry.opw.b)) < shoulderDistance };
    }

    IkResult ArmModel::Inverse(const Pose& target, const JointPositions& current) const
    {
        const auto nearest = opw.SolveNearest(baseInverse * target * toolInverse, ToVector(current), ikLimits);
        if (nearest.status == kinematics::SolutionStatus::unreachable)
            return IkResult{};
        if (!nearest.q.has_value())
            return IkResult{ std::nullopt, IkFailure::outsideLimits };
        return IkResult{ FromVector<foc::Radians>(*nearest.q), IkFailure::outsideLimits };
    }

    ArmModel::LinkFrames ArmModel::LinkPoses(const JointPositions& joints) const
    {
        const auto theta = ModelVector(joints);
        const auto chain = Chain(geometry.opw);
        LinkFrames frames{};
        auto pose = base;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            pose = pose * Pose{ math::RotationAboutAxis(chain[joint].axis, theta[joint]), chain[joint].origin };
            frames[joint] = pose;
        }
        return frames;
    }

    JointTorques ArmModel::GravityTorque(const JointPositions& joints) const
    {
        return ToJointTorques(dynamics.ComputeGravityTerms(ModelVector(joints)));
    }

    JointTorques ArmModel::InverseDynamics(const JointPositions& joints, const JointSpeeds& speeds, const JointAccelerations& accelerations) const
    {
        return ToJointTorques(dynamics.ComputeInverseDynamics(ModelVector(joints), ModelSpeeds(speeds), ModelAccelerations(accelerations)));
    }

    ArmModel::MassMatrix ArmModel::JointMassMatrix(const JointPositions& joints) const
    {
        return jointMap.Congruence(dynamics.ComputeMassMatrix(ModelVector(joints)));
    }

    ArmModel::JointVector ArmModel::ModelVector(const JointPositions& joints) const
    {
        return jointMap.Angles(joints);
    }

    const dynamics::EulerLagrangeDynamics<float, jointCount>& ArmModel::Dynamics() const
    {
        return dynamics;
    }

    const dynamics::InverseDynamicsModel<float, jointCount>& ArmModel::InverseModel() const
    {
        return dynamics;
    }

    const ArmModel::DynamicLinks& ArmModel::Links() const
    {
        return links;
    }

    const math::Vector3<float>& ArmModel::Gravity() const
    {
        return gravity;
    }

    ArmModel::JointVector ArmModel::ModelTorques(const JointTorques& torques) const
    {
        return jointMap.ToModel(torques);
    }

    JointPositions ArmModel::ToJointPositions(const JointVector& angles) const
    {
        return jointMap.Positions(angles);
    }

    JointSpeeds ArmModel::ToJointSpeeds(const JointVector& speeds) const
    {
        return jointMap.ToJoint<foc::RadiansPerSecond>(speeds);
    }

    ArmModel::JointVector ArmModel::ModelSpeeds(const JointSpeeds& speeds) const
    {
        return jointMap.ToModel(speeds);
    }

    ArmModel::JointVector ArmModel::ModelAccelerations(const JointAccelerations& accelerations) const
    {
        return jointMap.ToModel(accelerations);
    }

    JointTorques ArmModel::ToJointTorques(const JointVector& torques) const
    {
        return jointMap.ToJoint<foc::NewtonMeter>(torques);
    }

    math::Vector3<float> ArmModel::WristCentre(const JointVector& theta) const
    {
        const auto& p = geometry.opw;
        const auto elbow = theta[1] + theta[2] + forearmAngle;
        const auto x = p.a1 + p.c2 * math::Sin(theta[1]) + forearm * math::Sin(elbow);
        const auto z = p.c2 * math::Cos(theta[1]) + forearm * math::Cos(elbow);
        return math::RotationZ(theta[0]) * math::Vector3<float>{ x, p.b, z } + math::Vector3<float>{ 0.0f, 0.0f, p.c1 };
    }
}
