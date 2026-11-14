#ifndef CORE_ARM_ARM_CONTROLLER_ENGAGEMENT_HPP
#define CORE_ARM_ARM_CONTROLLER_ENGAGEMENT_HPP

#include "core/arm_controllers/implementations/ArmControllersImpl.hpp"
#include "core/lifecycle/interfaces/ArmLifecycle.hpp"

namespace arm
{
    class ArmControllerEngagement
        : private ArmLifecycleObserver
        , private ArmControllersObserver
    {
    public:
        ArmControllerEngagement(ArmLifecycle& lifecycle, ArmControllersImpl& controllers);

    private:
        void OnStateChanged(ArmState state) override;
        void OnActiveControllerChanged(std::optional<ArmControllerId> active) override;

        static bool Streams(ArmState state);
        void Follow(ArmState state);

        ArmControllersImpl& controllers;
    };
}

#endif
