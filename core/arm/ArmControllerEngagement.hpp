#pragma once

#include "core/arm_controllers/implementations/ArmControllersImpl.hpp"
#include "core/lifecycle/interfaces/ArmLifecycle.hpp"

namespace arm
{
    class ArmControllerEngagement
        : public MotionPermit
        , private ArmLifecycleObserver
        , private ArmControllersObserver
    {
    public:
        ArmControllerEngagement(ArmLifecycle& lifecycle, ArmControllersImpl& controllers);

        bool MotionPermitted() const override;

    private:
        void OnStateChanged(ArmState state) override;
        void OnActiveControllerChanged(std::optional<ArmControllerId> active) override;

        static bool Streams(ArmState state);
        void Follow(ArmState state);

        ArmControllersImpl& controllers;
    };
}
