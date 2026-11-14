#include "core/arm_controllers/implementations/ArmControllersImpl.hpp"
#include "infra/util/ReallyAssert.hpp"

namespace arm
{
    ArmControllersImpl::ArmControllersImpl(JointBus& bus, const JointMotion& motion)
        : bus(bus)
        , motion(motion)
    {}

    ArmControllerId ArmControllersImpl::Register(infra::BoundedConstString name, ArmController& controller)
    {
        really_assert(!entries.full() && !Find(name).has_value());
        entries.push_back(Entry{ name, &controller });
        return static_cast<ArmControllerId>(entries.size() - 1);
    }

    std::size_t ArmControllersImpl::Count() const
    {
        return entries.size();
    }

    infra::BoundedConstString ArmControllersImpl::Name(ArmControllerId id) const
    {
        return Registered(id) ? entries[id].name : infra::BoundedConstString{};
    }

    std::optional<ArmControllerId> ArmControllersImpl::Find(infra::BoundedConstString name) const
    {
        for (std::size_t id = 0; id != entries.size(); ++id)
            if (entries[id].name == name)
                return static_cast<ArmControllerId>(id);

        return std::nullopt;
    }

    bool ArmControllersImpl::Enabled(ArmControllerId id) const
    {
        return Registered(id) && entries[id].enabled;
    }

    std::optional<ArmControllerId> ArmControllersImpl::Active() const
    {
        return active;
    }

    bool ArmControllersImpl::Enable(ArmControllerId id)
    {
        if (!Registered(id))
            return false;

        entries[id].enabled = true;
        return true;
    }

    bool ArmControllersImpl::Disable(ArmControllerId id)
    {
        if (!Registered(id))
            return false;

        entries[id].enabled = false;
        if (active == id)
            Activate(std::nullopt);
        return true;
    }

    bool ArmControllersImpl::Select(ArmControllerId id)
    {
        if (!Enabled(id))
            return false;
        if (active == id)
            return true;

        auto& controller = *entries[id].controller;
        if (motion.Moving() && !controller.BumplessTransfer())
            return false;

        if (streaming)
            controller.Reset(Measured());
        Activate(id);
        return true;
    }

    void ArmControllersImpl::Engage()
    {
        const auto measured = Measured();
        heldReference = measured.positions;
        if (active.has_value())
            entries[*active].controller->Reset(measured);
        streaming = true;
    }

    void ArmControllersImpl::Disengage()
    {
        streaming = false;
    }

    bool ArmControllersImpl::Streaming() const
    {
        return streaming;
    }

    bool ArmControllersImpl::Step(const std::optional<JointPositions>& reference)
    {
        if (!streaming)
            return true;

        if (reference.has_value())
            heldReference = *reference;

        if (!active.has_value())
        {
            Disengage();
            return false;
        }

        const auto setpoints = entries[*active].controller->Step(Measured(), heldReference);
        if (!bus.SendPositions(setpoints.positions))
        {
            Disengage();
            return false;
        }

        lastSetpoints = setpoints;
        return true;
    }

    std::optional<JointSetpoints> ArmControllersImpl::LastSetpoints() const
    {
        return lastSetpoints;
    }

    bool ArmControllersImpl::Registered(ArmControllerId id) const
    {
        return id < entries.size();
    }

    MeasuredJoints ArmControllersImpl::Measured() const
    {
        MeasuredJoints measured;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto& reading = bus.Reading(joint);
            measured.positions[joint] = reading.position;
            measured.speeds[joint] = reading.speed;
        }
        return measured;
    }

    void ArmControllersImpl::Activate(std::optional<ArmControllerId> id)
    {
        active = id;
        NotifyObservers([id](auto& observer)
            {
                observer.OnActiveControllerChanged(id);
            });
    }
}
