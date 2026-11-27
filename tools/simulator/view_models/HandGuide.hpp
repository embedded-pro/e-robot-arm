#pragma once

#include "core/arm_model/ArmModel.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ScenePicking.hpp"
#include "ui/core/Callback.hpp"
#include <array>
#include <optional>
#include <string>

namespace simulator
{
    struct GuideArrow
    {
        ScenePoint from{};
        ScenePoint to{};

        bool operator==(const GuideArrow& other) const = default;
    };

    class HandGuide
    {
    public:
        static constexpr float stiffness{ 400.0f };
        static constexpr float maximumForce{ 100.0f };
        static constexpr float maximumTorque{ 200.0f };

        explicit HandGuide(const ui::Callback<void(const arm_host::ArmCommand& command)>& post);

        static bool Offered(const arm_host::ArmSnapshot& snapshot);

        bool Grab(int pickedJoint, const ScenePoint& hit, const arm_host::ArmSnapshot& snapshot);
        void Drag(const SceneRay& ray, const arm_host::ArmSnapshot& snapshot);
        void Release();

        bool Active() const;
        const std::optional<GuideArrow>& Arrow() const;
        float Force() const;
        std::string Label() const;

    private:
        static arm::JointPositions Measured(const arm_host::ArmSnapshot& snapshot);
        math::Vector3<float> GrabbedPoint(const arm::JointPositions& joints) const;
        arm::JointTorques Torques(const math::Vector3<float>& force, const arm::JointPositions& joints) const;

        ui::Callback<void(const arm_host::ArmCommand& command)> post;
        std::optional<arm::ArmModel> model;
        std::optional<std::size_t> link;
        math::Vector3<float> local{};
        std::optional<GuideArrow> arrow;
        float force{ 0.0f };
    };
}
