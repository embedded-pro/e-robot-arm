#ifndef CORE_JOINT_BUS_INTERFACES_JOINT_BUS_HPP
#define CORE_JOINT_BUS_INTERFACES_JOINT_BUS_HPP

#include "e-foc-protocol/client/FocMotorCategoryClient.hpp"
#include "e-foc-protocol/contract/FocMotorMessages.hpp"
#include "e-foc-protocol/foc/Units.hpp"
#include "infra/util/Observer.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace arm
{
    inline constexpr std::size_t jointCount{ 6 };

    using JointNodeIds = std::array<uint16_t, jointCount>;
    using JointPositions = std::array<foc::Radians, jointCount>;

    struct JointLimit
    {
        foc::Radians lower;
        foc::Radians upper;
    };

    using JointLimits = std::array<JointLimit, jointCount>;

    inline bool WithinLimits(const JointLimits& limits, const JointPositions& positions)
    {
        for (std::size_t joint = 0; joint != jointCount; ++joint)
            if (!(limits[joint].lower <= positions[joint] && positions[joint] <= limits[joint].upper))
                return false;

        return true;
    }

    struct JointReading
    {
        foc::Radians position{ 0.0f };
        foc::RadiansPerSecond speed{ 0.0f };
        can::FocMotorState state{ can::FocMotorState::idle };
        can::FocFaultCode fault{ can::FocFaultCode::none };
    };

    class JointBus;

    class JointBusObserver
        : public infra::SingleObserver<JointBusObserver, JointBus>
    {
    public:
        using infra::SingleObserver<JointBusObserver, JointBus>::SingleObserver;

        virtual void OnContractsResolved(can::ContractCompatibility compatibility) = 0;
        virtual void OnJointFault(std::size_t joint, can::FocFaultCode fault) = 0;
        virtual void OnJointSilent(std::size_t joint) = 0;

    protected:
        ~JointBusObserver() = default;
    };

    class JointBus
        : public infra::Subject<JointBusObserver>
    {
    public:
        JointBus() = default;
        JointBus(const JointBus& other) = delete;
        JointBus& operator=(const JointBus& other) = delete;

        virtual bool QueryContracts() = 0;
        virtual can::ContractCompatibility Contracts() const = 0;

        virtual bool SelectMode(can::FocMotorMode mode) = 0;
        virtual bool Start() = 0;
        virtual bool Stop() = 0;
        virtual bool ClearFault() = 0;
        virtual bool EmergencyStop() = 0;
        virtual bool SendPositions(const JointPositions& positions) = 0;
        virtual bool RequestTelemetry() = 0;

        virtual const JointReading& Reading(std::size_t joint) const = 0;

    protected:
        ~JointBus() = default;
    };
}

#endif
