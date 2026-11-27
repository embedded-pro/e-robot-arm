#pragma once

#include "core/arm/ArmConfig.hpp"
#include <cstdint>
#include <optional>
#include <variant>

namespace arm
{
    namespace tuningLimit
    {
        inline constexpr float bandwidth{ 500.0f };
        inline constexpr float dampingRatio{ 5.0f };
        inline constexpr float singularityDamping{ 1.0f };
        inline constexpr float lambda{ 100.0f };
        inline constexpr float adaptationGain{ 10000.0f };
    }

    enum class TuningField : uint8_t
    {
        bandwidth,
        dampingRatio,
        stiffness,
        translationalStiffness,
        rotationalStiffness,
        singularityDamping,
        lambda,
        adaptationGain
    };

    struct TuningViolation
    {
        TuningField field;
        std::optional<float> maximum;

        bool operator==(const TuningViolation& other) const = default;
    };

    enum class RetuneResult : uint8_t
    {
        applied,
        notTunable,
        outOfRange,
        moving
    };

    enum class PresetResult : uint8_t
    {
        saved,
        replaced,
        notTunable,
        badName,
        full
    };

    bool Tunable(ArmControllerKind kind);
    bool TuningFits(ArmControllerKind kind, const ArmControllerTuning& tuning);
    std::optional<TuningViolation> CheckTuning(const ArmControllerTuning& tuning);
    ArmControllerTuning TuningOf(const ArmConfig& config, ArmControllerKind kind);
    void StoreTuning(ArmConfig& config, ArmControllerKind kind, const ArmControllerTuning& tuning);
    PresetResult SavePreset(ArmConfig& config, ArmControllerKind kind, infra::BoundedConstString name);
}
