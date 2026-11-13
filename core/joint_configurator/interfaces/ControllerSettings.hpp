#ifndef CORE_JOINT_CONFIGURATOR_INTERFACES_CONTROLLER_SETTINGS_HPP
#define CORE_JOINT_CONFIGURATOR_INTERFACES_CONTROLLER_SETTINGS_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "e-foc-protocol/contract/FocMotorMessages.hpp"
#include "numerical/math/Math.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace arm
{
    enum class Gain : uint8_t
    {
        currentBandwidth,
        currentSwitchingGain,
        currentBoundaryLayer,
        currentTwoStepDeadbeat,
        speedBandwidth,
        speedErrorWeight,
        speedIntegralWeight,
        speedObserverBandwidthRatio,
        speedReferenceTimeConstant,
        positionBandwidth,
        positionErrorWeight,
        positionSpeedErrorWeight,
        positionIntegralWeight,
        positionReferenceTimeConstant
    };

    inline constexpr std::size_t gainCount{ 14 };

    inline constexpr std::array<can::FocParameter, gainCount> gainParameters{ can::FocParameter::currentBandwidth, can::FocParameter::currentSwitchingGain,
        can::FocParameter::currentBoundaryLayer, can::FocParameter::currentTwoStepDeadbeat, can::FocParameter::speedBandwidth, can::FocParameter::speedErrorWeight,
        can::FocParameter::speedIntegralWeight, can::FocParameter::speedObserverBandwidthRatio, can::FocParameter::speedReferenceTimeConstant,
        can::FocParameter::positionBandwidth, can::FocParameter::positionErrorWeight, can::FocParameter::positionSpeedErrorWeight,
        can::FocParameter::positionIntegralWeight, can::FocParameter::positionReferenceTimeConstant };

    struct ControllerSettings
    {
        can::FocCurrentAlgorithm current{ can::FocCurrentAlgorithm::pid };
        can::FocSpeedAlgorithm speed{ can::FocSpeedAlgorithm::pid };
        can::FocPositionAlgorithm position{ can::FocPositionAlgorithm::pid };
        std::array<float, gainCount> gains{ 6283.185307f, 0.2f, 0.5f, 0.0f, 188.495559f, 1.0f, 0.1f, 5.0f, 0.0053051f, 18.8495559f, 1.0f, 0.1f, 0.05f,
            0.053051f };

        float& operator[](Gain gain)
        {
            return gains[static_cast<std::size_t>(gain)];
        }

        float operator[](Gain gain) const
        {
            return gains[static_cast<std::size_t>(gain)];
        }
    };

    using JointControllers = std::array<std::optional<ControllerSettings>, jointCount>;

    struct ControllerReading
    {
        can::FocMotorMode mode{ can::FocMotorMode::torque };
        ControllerSettings settings{};
    };

    struct ControllerMismatches
    {
        bool current{ false };
        bool speed{ false };
        bool position{ false };
        std::array<bool, gainCount> gains{};

        bool Any() const
        {
            return current || speed || position || std::ranges::find(gains, true) != gains.end();
        }
    };

    inline bool GainsMatch(float expected, float actual)
    {
        return math::Abs(expected - actual) <= 1.0e-4f * std::max(1.0f, math::Abs(expected));
    }

    inline ControllerMismatches Compare(const ControllerSettings& expected, const ControllerSettings& actual)
    {
        ControllerMismatches mismatches{ expected.current != actual.current, expected.speed != actual.speed, expected.position != actual.position, {} };
        for (std::size_t gain = 0; gain != gainCount; ++gain)
            mismatches.gains[gain] = !GainsMatch(expected.gains[gain], actual.gains[gain]);
        return mismatches;
    }

    inline bool Uses(const ControllerSettings& settings, Gain gain)
    {
        using enum Gain;
        switch (gain)
        {
            case currentTwoStepDeadbeat:
                return settings.current == can::FocCurrentAlgorithm::deadbeat;
            case currentSwitchingGain:
            case currentBoundaryLayer:
                return settings.current == can::FocCurrentAlgorithm::slidingMode;
            case speedErrorWeight:
            case speedIntegralWeight:
                return settings.speed == can::FocSpeedAlgorithm::lqi;
            case speedObserverBandwidthRatio:
                return settings.speed == can::FocSpeedAlgorithm::adrc;
            case speedReferenceTimeConstant:
                return settings.speed == can::FocSpeedAlgorithm::twoDof;
            case positionErrorWeight:
            case positionSpeedErrorWeight:
                return settings.position == can::FocPositionAlgorithm::lqr || settings.position == can::FocPositionAlgorithm::lqi;
            case positionIntegralWeight:
                return settings.position == can::FocPositionAlgorithm::lqi;
            case positionReferenceTimeConstant:
                return settings.position == can::FocPositionAlgorithm::twoDof;
            default:
                return true;
        }
    }
}

#endif
