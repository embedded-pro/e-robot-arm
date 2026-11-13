#ifndef CORE_JOINT_CONFIGURATOR_INTERFACES_MOTOR_SETTINGS_HPP
#define CORE_JOINT_CONFIGURATOR_INTERFACES_MOTOR_SETTINGS_HPP

#include "core/joint_bus/interfaces/JointBus.hpp"
#include "core/joint_configurator/interfaces/ControllerSettings.hpp"
#include "e-foc-protocol/contract/FocMotorMessages.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace arm
{
    enum class MotorParameter : uint8_t
    {
        resistance,
        inductance,
        polePairs,
        fluxLinkage,
        inertia,
        viscousFriction,
        coulombFriction,
        continuousCurrent,
        peakCurrent,
        peakCurrentTime,
        overspeedLimit
    };

    inline constexpr std::size_t motorParameterCount{ 11 };
    inline constexpr std::size_t motorModelCount{ 7 };

    inline constexpr std::array<can::FocParameter, motorParameterCount> motorParameters{ can::FocParameter::resistance, can::FocParameter::inductance,
        can::FocParameter::polePairs, can::FocParameter::fluxLinkage, can::FocParameter::inertia, can::FocParameter::viscousFriction,
        can::FocParameter::frictionCoulomb, can::FocParameter::continuousCurrent, can::FocParameter::peakCurrent, can::FocParameter::peakCurrentTime,
        can::FocParameter::overspeedLimit };

    struct MotorUnit
    {
        std::string_view key;
        std::string_view label;
        std::string_view unit;
        float scale;
    };

    inline constexpr std::array<MotorUnit, motorParameterCount> motorUnits{ {
        { "resistanceOhm", "Resistance", "Ω", 1.0f },
        { "inductanceMilliHenry", "Inductance", "mH", 1.0f },
        { "polePairs", "Pole pairs", "", 1.0f },
        { "fluxLinkageMilliWeber", "Flux linkage", "mWb", 1.0e3f },
        { "inertiaGramCm2", "Rotor inertia", "g·cm²", 1.0e7f },
        { "viscousFrictionMicroNmSPerRad", "Viscous friction", "µN·m·s/rad", 1.0e6f },
        { "coulombFrictionMilliNm", "Coulomb friction", "mN·m", 1.0e3f },
        { "continuousCurrentA", "Continuous current", "A", 1.0f },
        { "peakCurrentA", "Peak current", "A", 1.0f },
        { "peakCurrentTimeS", "Peak-current time", "s", 1.0f },
        { "overspeedLimitRadPerS", "Overspeed limit", "rad/s", 1.0f },
    } };

    struct MotorSettings
    {
        std::array<float, motorParameterCount> values{};

        float& operator[](MotorParameter parameter)
        {
            return values[static_cast<std::size_t>(parameter)];
        }

        float operator[](MotorParameter parameter) const
        {
            return values[static_cast<std::size_t>(parameter)];
        }
    };

    struct MotorPreset
    {
        std::string_view key;
        std::string_view name;
        MotorSettings settings;
    };

    inline constexpr std::array<MotorPreset, 2> motorPresets{ {
        { "teknic-m2310p-ln-04k", "Teknic M-2310P-LN-04K", { { 0.36f, 0.2f, 4.0f, 0.0064f, 7.06e-6f, 1.5e-5f, 0.0037f, 7.1f, 20.0f, 1.0f, 754.0f } } },
        { "anaheim-bly172s-24v-4000", "Anaheim BLY172S-24V-4000", { { 0.405f, 0.64f, 4.0f, 0.00544f, 4.8e-6f, 1.0e-5f, 0.0037f, 3.9f, 10.0f, 1.0f, 503.0f } } },
    } };

    struct JointMotor
    {
        std::optional<std::size_t> preset;
        MotorSettings settings;
    };

    using JointMotors = std::array<std::optional<JointMotor>, jointCount>;

    struct MotorMismatches
    {
        std::array<bool, motorParameterCount> values{};

        bool Any() const
        {
            return std::ranges::find(values, true) != values.end();
        }
    };

    inline bool MotorValuesMatch(float expected, float actual)
    {
        return math::Abs(expected - actual) <= 1.0e-4f * std::max(math::Abs(expected), 1.0e-12f);
    }

    inline MotorMismatches Compare(const MotorSettings& expected, const MotorSettings& actual)
    {
        MotorMismatches mismatches{};
        for (std::size_t parameter = 0; parameter != motorParameterCount; ++parameter)
            mismatches.values[parameter] = !MotorValuesMatch(expected.values[parameter], actual.values[parameter]);
        return mismatches;
    }
}

#endif
