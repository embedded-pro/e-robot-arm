#ifndef CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_ARM_CONTROLLERS_IMPL_HPP
#define CORE_ARM_CONTROLLERS_IMPLEMENTATIONS_ARM_CONTROLLERS_IMPL_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_controllers/interfaces/ArmControllers.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "infra/util/BoundedVector.hpp"
#include <optional>

namespace arm
{
    inline constexpr std::size_t maxArmControllers{ 4 };

    class ArmControllersImpl
        : public ArmControllers
    {
    public:
        ArmControllersImpl(JointBus& bus, const JointMotion& motion);

        ArmControllerId Register(infra::BoundedConstString name, ArmController& controller);

        std::size_t Count() const override;
        infra::BoundedConstString Name(ArmControllerId id) const override;
        std::optional<ArmControllerId> Find(infra::BoundedConstString name) const override;
        bool Enabled(ArmControllerId id) const override;
        std::optional<ArmControllerId> Active() const override;
        std::optional<JointSetpoints> LastSetpoints() const override;

        bool Enable(ArmControllerId id) override;
        bool Disable(ArmControllerId id) override;
        bool Select(ArmControllerId id) override;

        void Engage();
        void Disengage();
        bool Streaming() const;
        bool Step(const std::optional<JointPositions>& reference);

    private:
        struct Entry
        {
            infra::BoundedConstString name;
            ArmController* controller;
            bool enabled{ false };
        };

        bool Registered(ArmControllerId id) const;
        MeasuredJoints Measured() const;
        void Activate(std::optional<ArmControllerId> id);

        JointBus& bus;
        const JointMotion& motion;
        infra::BoundedVector<Entry>::WithMaxSize<maxArmControllers> entries;
        std::optional<ArmControllerId> active;
        bool streaming{ false };
        JointPositions heldReference{};
        std::optional<JointSetpoints> lastSetpoints;
    };
}

#endif
