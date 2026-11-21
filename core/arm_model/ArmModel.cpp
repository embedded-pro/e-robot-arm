#include "core/arm_model/ArmModel.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include "numerical/solvers/LuDecomposition.hpp"
#include "robotics/kinematics/GeometricJacobian.hpp"
#include <limits>
#include <numbers>

namespace arm
{
    namespace
    {
        constexpr float twoPi{ 2.0f * std::numbers::pi_v<float> };

        math::Matrix3<float> RotationX(float angle)
        {
            const auto c = math::Cos(angle);
            const auto s = math::Sin(angle);
            return math::Matrix3<float>{ { 1.0f, 0.0f, 0.0f }, { 0.0f, c, -s }, { 0.0f, s, c } };
        }

        math::Matrix3<float> RotationY(float angle)
        {
            const auto c = math::Cos(angle);
            const auto s = math::Sin(angle);
            return math::Matrix3<float>{ { c, 0.0f, s }, { 0.0f, 1.0f, 0.0f }, { -s, 0.0f, c } };
        }

        math::Matrix3<float> RotationZ(float angle)
        {
            const auto c = math::Cos(angle);
            const auto s = math::Sin(angle);
            return math::Matrix3<float>{ { c, -s, 0.0f }, { s, c, 0.0f }, { 0.0f, 0.0f, 1.0f } };
        }

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

        math::Matrix3<float> ParallelAxis(float mass, const math::Vector3<float>& offset)
        {
            const auto outer = offset * offset.Transpose();
            return (math::Matrix3<float>::Identity() * math::DotProduct(offset, offset) - outer) * mass;
        }

        dynamics::JointLink<float> WithPayload(dynamics::JointLink<float> link, const MassProperties& payload, float flange)
        {
            const auto total = link.mass + payload.mass;
            if (!(total > 0.0f))
                return link;

            const auto payloadCentre = Point(payload.centre) + math::Vector3<float>{ 0.0f, 0.0f, flange };
            const auto centre = (link.jointToCoM * link.mass + payloadCentre * payload.mass) * (1.0f / total);
            link.inertia = link.inertia + ParallelAxis(link.mass, link.jointToCoM - centre) + Tensor(payload.inertia) + ParallelAxis(payload.mass, payloadCentre - centre);
            link.mass = total;
            link.jointToCoM = centre;
            return link;
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
        return { RotationZ(placement.yaw) * RotationY(placement.pitch) * RotationX(placement.roll), math::Vector3<float>{ placement.x, placement.y, placement.z } };
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
        , limits(limits)
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

        const auto shoulder = RotationZ(theta[0]);
        const auto upper = shoulder * RotationY(theta[1] + theta[2]);
        const auto wrist4 = upper * RotationZ(theta[3]);
        const auto wrist5 = wrist4 * RotationY(theta[4]);
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
        const auto solutions = opw.Solve(baseInverse * target * toolInverse);

        IkResult result{};
        float best{ std::numeric_limits<float>::max() };
        for (const auto& solution : solutions)
        {
            if (!solution.has_value())
                continue;

            result.failure = IkFailure::outsideLimits;
            const auto candidate = WithinLimits(solution->q, current);
            if (!candidate.has_value())
                continue;

            const JointVector difference{ ToVector(*candidate) - ToVector(current) };
            const auto distance = math::DotProduct(difference, difference);

            if (distance < best)
            {
                best = distance;
                result.joints = candidate;
            }
        }

        return result;
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
        return RotationZ(theta[0]) * math::Vector3<float>{ x, p.b, z } + math::Vector3<float>{ 0.0f, 0.0f, p.c1 };
    }

    std::optional<JointPositions> ArmModel::WithinLimits(const JointVector& solution, const JointPositions& current) const
    {
        JointPositions joints{};
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            std::optional<float> chosen;
            for (const auto shift : { 0.0f, twoPi, -twoPi })
            {
                const auto value = solution[joint] + shift;
                const bool inside = limits[joint].lower.Value() <= value && value <= limits[joint].upper.Value();
                if (inside && (!chosen.has_value() || math::Abs(value - current[joint].Value()) < math::Abs(*chosen - current[joint].Value())))
                    chosen = value;
            }

            if (!chosen.has_value())
                return std::nullopt;
            joints[joint] = foc::Radians{ *chosen };
        }

        return joints;
    }
}
