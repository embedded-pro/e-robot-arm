#include "tools/simulator/view_models/HandGuide.hpp"
#include <algorithm>
#include <cmath>
#include <format>

namespace simulator
{
    namespace
    {
        constexpr float centimetres{ 100.0f };
        constexpr float jointStep{ 1.0e-3f };

        math::Vector3<float> ToBase(const ScenePoint& point)
        {
            return math::Vector3<float>{ point[0] / centimetres, -point[2] / centimetres, point[1] / centimetres };
        }

        ScenePoint ToScenePoint(const math::Vector3<float>& point)
        {
            return { point[0] * centimetres, point[2] * centimetres, -point[1] * centimetres };
        }

        float Dot(const ScenePoint& a, const ScenePoint& b)
        {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        ScenePoint Nearest(const SceneRay& ray, const ScenePoint& point)
        {
            const auto length = Dot(ray.direction, ray.direction);
            const ScenePoint offset{ point[0] - ray.origin[0], point[1] - ray.origin[1], point[2] - ray.origin[2] };
            const auto along = length > 0.0f ? Dot(offset, ray.direction) / length : 0.0f;
            return { ray.origin[0] + along * ray.direction[0], ray.origin[1] + along * ray.direction[1], ray.origin[2] + along * ray.direction[2] };
        }

        float Norm(const math::Vector3<float>& vector)
        {
            return std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
        }
    }

    HandGuide::HandGuide(const ui::Callback<void(const arm_host::ArmCommand& command)>& post)
        : post(post)
    {}

    bool HandGuide::Offered(const arm_host::ArmSnapshot& snapshot)
    {
        return snapshot.targetReady && snapshot.disturbances.has_value() && snapshot.disturbances->collision;
    }

    bool HandGuide::Grab(int pickedJoint, const ScenePoint& hit, const arm_host::ArmSnapshot& snapshot)
    {
        if (pickedJoint < 0 || pickedJoint >= static_cast<int>(arm::jointCount) || !Offered(snapshot))
            return false;

        model.emplace(snapshot.config.geometry, snapshot.config.mass, snapshot.config.positionLimits);
        link = static_cast<std::size_t>(pickedJoint);
        local = model->LinkPoses(Measured(snapshot))[*link].Inverse().Apply(ToBase(hit));
        arrow.reset();
        force = 0.0f;
        return true;
    }

    void HandGuide::Drag(const SceneRay& ray, const arm_host::ArmSnapshot& snapshot)
    {
        if (!Active())
            return;
        if (!Offered(snapshot))
        {
            Release();
            return;
        }

        const auto joints = Measured(snapshot);
        const auto point = GrabbedPoint(joints);
        const auto from = ToScenePoint(point);
        const auto to = Nearest(ray, from);
        auto pull = (ToBase(to) - point) * stiffness;
        force = Norm(pull);
        if (force > maximumForce)
        {
            pull = pull * (maximumForce / force);
            force = maximumForce;
        }
        arrow = GuideArrow{ from, to };
        post(arm_host::GuideArm{ Torques(pull, joints) });
    }

    void HandGuide::Release()
    {
        if (!Active())
            return;
        link.reset();
        arrow.reset();
        force = 0.0f;
        post(arm_host::GuideArm{});
    }

    bool HandGuide::Active() const
    {
        return link.has_value();
    }

    const std::optional<GuideArrow>& HandGuide::Arrow() const
    {
        return arrow;
    }

    float HandGuide::Force() const
    {
        return force;
    }

    std::string HandGuide::Label() const
    {
        if (!Active())
            return {};
        return std::format("J{} · guided · {:.0f} N", *link + 1, force);
    }

    arm::JointPositions HandGuide::Measured(const arm_host::ArmSnapshot& snapshot)
    {
        arm::JointPositions joints{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            joints[joint] = snapshot.readings[joint].position;
        return joints;
    }

    math::Vector3<float> HandGuide::GrabbedPoint(const arm::JointPositions& joints) const
    {
        return model->LinkPoses(joints)[*link].Apply(local);
    }

    arm::JointTorques HandGuide::Torques(const math::Vector3<float>& pull, const arm::JointPositions& joints) const
    {
        const auto point = GrabbedPoint(joints);
        arm::JointTorques torques{};
        for (std::size_t joint = 0; joint <= *link; ++joint)
        {
            auto moved = joints;
            moved[joint] = foc::Radians{ moved[joint].Value() + jointStep };
            const auto column = (GrabbedPoint(moved) - point) * (1.0f / jointStep);
            const auto torque = column[0] * pull[0] + column[1] * pull[1] + column[2] * pull[2];
            torques[joint] = foc::NewtonMeter{ std::clamp(torque, -maximumTorque, maximumTorque) };
        }
        return torques;
    }
}
