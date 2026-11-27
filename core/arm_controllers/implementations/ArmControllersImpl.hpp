#pragma once

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "core/arm_controllers/interfaces/ArmControllers.hpp"
#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/motion/interfaces/JointMotion.hpp"
#include "infra/util/BoundedVector.hpp"
#include <optional>

namespace arm
{
    inline constexpr std::size_t maxArmControllers{ 10 };

    class ArmControllersImpl
        : public ArmControllers
    {
    public:
        static constexpr uint32_t maxExtrapolatedPeriods{ 2 };

        ArmControllersImpl(JointBus& bus, const JointMotion& motion, infra::Duration period = {});

        ArmControllerId Register(infra::BoundedConstString name, ArmController& controller, bool passive = false);

        std::size_t Count() const override;
        infra::BoundedConstString Name(ArmControllerId id) const override;
        std::optional<ArmControllerId> Find(infra::BoundedConstString name) const override;
        bool Enabled(ArmControllerId id) const override;
        bool Passive(ArmControllerId id) const override;
        bool ActivePassive() const;
        std::optional<ArmControllerId> Active() const override;
        std::optional<JointSetpoints> LastSetpoints() const override;
        std::optional<JointPositions> Reference() const override;
        std::optional<JointReference> TrackedReference() const;

        bool Enable(ArmControllerId id) override;
        bool Disable(ArmControllerId id) override;
        bool Select(ArmControllerId id) override;

        void Engage();
        void Hold();
        void Disengage();
        void Resume();
        bool Streaming() const;
        bool StreamingPassive() const;
        bool Step(const std::optional<JointReference>& reference);

    private:
        struct Entry
        {
            infra::BoundedConstString name;
            ArmController* controller;
            bool enabled{ false };
            bool passive{ false };
        };

        bool Registered(ArmControllerId id) const;
        MeasuredJoints Measured() const;
        void Activate(std::optional<ArmControllerId> id);

        JointBus& bus;
        const JointMotion& motion;
        float periodSeconds;
        infra::BoundedVector<Entry>::WithMaxSize<maxArmControllers> entries;
        std::optional<ArmControllerId> active;
        bool streaming{ false };
        bool holding{ false };
        JointReference heldReference{};
        std::optional<JointSetpoints> lastSetpoints;
    };
}
