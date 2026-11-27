#include "core/arm_controllers/implementations/ArmControllersImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>
#include <chrono>

namespace arm
{
    ArmControllersImpl::ArmControllersImpl(JointBus& bus, const JointMotion& motion, infra::Duration period)
        : bus(bus)
        , motion(motion)
        , periodSeconds(std::chrono::duration<float>{ period }.count())
    {}

    ArmControllerId ArmControllersImpl::Register(infra::BoundedConstString name, ArmController& controller, bool passive)
    {
        really_assert(!entries.full() && !Find(name).has_value());
        entries.push_back(Entry{ name, &controller, false, passive });
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

    bool ArmControllersImpl::Passive(ArmControllerId id) const
    {
        return Registered(id) && entries[id].passive;
    }

    bool ArmControllersImpl::ActivePassive() const
    {
        return active.has_value() && entries[*active].passive;
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
        if (!Registered(id) || (streaming && active == id))
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
            controller.Reset(Measured(), heldReference);
        Activate(id);
        return true;
    }

    void ArmControllersImpl::Engage()
    {
        Hold();
        holding = false;
        streaming = true;
    }

    void ArmControllersImpl::Hold()
    {
        holding = true;
        const auto measured = Measured();
        heldReference = AtRest(measured.positions);
        if (active.has_value())
            entries[*active].controller->Reset(measured, heldReference);
    }

    void ArmControllersImpl::Disengage()
    {
        streaming = false;
        holding = false;
    }

    void ArmControllersImpl::Resume()
    {
        holding = false;
    }

    bool ArmControllersImpl::StreamingPassive() const
    {
        return streaming && !holding && ActivePassive();
    }

    bool ArmControllersImpl::Streaming() const
    {
        return streaming;
    }

    bool ArmControllersImpl::Step(const std::optional<JointReference>& reference)
    {
        if (!streaming)
            return true;

        if (!active.has_value())
        {
            Disengage();
            return false;
        }

        const auto measured = Measured();
        if (ActivePassive() && !holding)
            heldReference = AtRest(measured.positions);
        else if (reference.has_value())
            heldReference = *reference;
        else
            heldReference = AtRest(heldReference.positions);

        const auto setpoints = entries[*active].controller->Step(measured, heldReference);
        if (!bus.SendTorques(setpoints.torques))
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

    std::optional<JointReference> ArmControllersImpl::TrackedReference() const
    {
        if (!streaming)
            return std::nullopt;
        return heldReference;
    }

    std::optional<JointPositions> ArmControllersImpl::Reference() const
    {
        if (!streaming)
            return std::nullopt;
        return heldReference.positions;
    }

    bool ArmControllersImpl::Registered(ArmControllerId id) const
    {
        return id < entries.size();
    }

    MeasuredJoints ArmControllersImpl::Measured() const
    {
        MeasuredJoints measured;
        const auto current = periodSeconds > 0.0f ? bus.Period() : 0;
        for (std::size_t joint = 0; joint != jointCount; ++joint)
        {
            const auto& reading = bus.Reading(joint);
            const auto age = reading.period != 0 && current > reading.period ? std::min(current - reading.period, maxExtrapolatedPeriods) : 0;
            measured.positions[joint] = foc::Radians{ reading.position.Value() + reading.speed.Value() * periodSeconds * static_cast<float>(age) };
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
