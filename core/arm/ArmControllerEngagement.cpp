#include "core/arm/ArmControllerEngagement.hpp"

namespace arm
{
    ArmControllerEngagement::ArmControllerEngagement(ArmLifecycle& lifecycle, ArmControllersImpl& controllers)
        : ArmLifecycleObserver(lifecycle)
        , ArmControllersObserver(controllers)
        , controllers(controllers)
    {}

    void ArmControllerEngagement::OnStateChanged(ArmState state)
    {
        Follow(state);
        if (state == ArmState::holding && controllers.Streaming())
            controllers.Hold();
    }

    void ArmControllerEngagement::OnActiveControllerChanged(std::optional<ArmControllerId> active)
    {
        if (active.has_value())
            Follow(ArmLifecycleObserver::Subject().State());
    }

    bool ArmControllerEngagement::Streams(ArmState state)
    {
        return state == ArmState::enabled || state == ArmState::moving || state == ArmState::holding;
    }

    void ArmControllerEngagement::Follow(ArmState state)
    {
        if (!Streams(state))
            controllers.Disengage();
        else if (!controllers.Streaming() && controllers.Active().has_value())
            controllers.Engage();
    }
}
