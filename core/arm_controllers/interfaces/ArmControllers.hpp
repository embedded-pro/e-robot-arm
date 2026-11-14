#ifndef CORE_ARM_CONTROLLERS_INTERFACES_ARM_CONTROLLERS_HPP
#define CORE_ARM_CONTROLLERS_INTERFACES_ARM_CONTROLLERS_HPP

#include "core/arm_controllers/interfaces/ArmController.hpp"
#include "infra/util/BoundedString.hpp"
#include "infra/util/Observer.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>

namespace arm
{
    using ArmControllerId = uint8_t;

    class ArmControllers;

    class ArmControllersObserver
        : public infra::Observer<ArmControllersObserver, ArmControllers>
    {
    public:
        using infra::Observer<ArmControllersObserver, ArmControllers>::Observer;

        virtual void OnActiveControllerChanged(std::optional<ArmControllerId> active) = 0;

    protected:
        ~ArmControllersObserver() = default;
    };

    class ArmControllers
        : public infra::Subject<ArmControllersObserver>
    {
    public:
        ArmControllers() = default;
        ArmControllers(const ArmControllers& other) = delete;
        ArmControllers& operator=(const ArmControllers& other) = delete;

        virtual std::size_t Count() const = 0;
        virtual infra::BoundedConstString Name(ArmControllerId id) const = 0;
        virtual std::optional<ArmControllerId> Find(infra::BoundedConstString name) const = 0;
        virtual bool Enabled(ArmControllerId id) const = 0;
        virtual std::optional<ArmControllerId> Active() const = 0;
        virtual std::optional<JointSetpoints> LastSetpoints() const = 0;

        virtual bool Enable(ArmControllerId id) = 0;
        virtual bool Disable(ArmControllerId id) = 0;
        virtual bool Select(ArmControllerId id) = 0;

    protected:
        ~ArmControllers() = default;
    };
}

#endif
