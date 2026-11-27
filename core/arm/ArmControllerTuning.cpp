#include "core/arm/ArmControllerTuning.hpp"
#include <algorithm>
#include <cmath>

namespace arm
{
    namespace
    {
        bool Within(float value, std::optional<float> maximum)
        {
            return std::isfinite(value) && value > 0.0f && (!maximum.has_value() || value <= *maximum);
        }

        class Checker
        {
        public:
            std::optional<TuningViolation> operator()(const BandwidthTuning& tuning) const
            {
                return First({ { TuningField::bandwidth, tuning.bandwidth, tuningLimit::bandwidth }, { TuningField::dampingRatio, tuning.dampingRatio, tuningLimit::dampingRatio } });
            }

            std::optional<TuningViolation> operator()(const JointImpedanceTuning& tuning) const
            {
                if (tuning.stiffness.has_value())
                    for (const auto stiffness : *tuning.stiffness)
                        if (!Within(stiffness, std::nullopt))
                            return TuningViolation{ TuningField::stiffness, std::nullopt };
                return First({ { TuningField::dampingRatio, tuning.dampingRatio, tuningLimit::dampingRatio } });
            }

            std::optional<TuningViolation> operator()(const CartesianImpedanceTuning& tuning) const
            {
                return First({ { TuningField::translationalStiffness, tuning.translationalStiffness, std::nullopt },
                    { TuningField::rotationalStiffness, tuning.rotationalStiffness, std::nullopt }, { TuningField::dampingRatio, tuning.dampingRatio, tuningLimit::dampingRatio } });
            }

            std::optional<TuningViolation> operator()(const OperationalSpaceTuning& tuning) const
            {
                return First({ { TuningField::bandwidth, tuning.bandwidth, tuningLimit::bandwidth }, { TuningField::dampingRatio, tuning.dampingRatio, tuningLimit::dampingRatio },
                    { TuningField::singularityDamping, tuning.singularityDamping, tuningLimit::singularityDamping } });
            }

            std::optional<TuningViolation> operator()(const SlotineLiTuning& tuning) const
            {
                return First({ { TuningField::lambda, tuning.lambda, tuningLimit::lambda }, { TuningField::bandwidth, tuning.bandwidth, tuningLimit::bandwidth },
                    { TuningField::adaptationGain, tuning.adaptationGain, tuningLimit::adaptationGain } });
            }

        private:
            struct Value
            {
                TuningField field;
                float value;
                std::optional<float> maximum;
            };

            static std::optional<TuningViolation> First(std::initializer_list<Value> values)
            {
                for (const auto& value : values)
                    if (!Within(value.value, value.maximum))
                        return TuningViolation{ value.field, value.maximum };
                return std::nullopt;
            }
        };
    }

    bool Tunable(ArmControllerKind kind)
    {
        return kind != ArmControllerKind::free && kind != ArmControllerKind::gravityFloat;
    }

    bool TuningFits(ArmControllerKind kind, const ArmControllerTuning& tuning)
    {
        switch (kind)
        {
            case ArmControllerKind::gravityPd:
            case ArmControllerKind::computedTorque:
            case ArmControllerKind::gravityPid:
                return std::holds_alternative<BandwidthTuning>(tuning);
            case ArmControllerKind::jointImpedance:
                return std::holds_alternative<JointImpedanceTuning>(tuning);
            case ArmControllerKind::cartesianImpedance:
                return std::holds_alternative<CartesianImpedanceTuning>(tuning);
            case ArmControllerKind::operationalSpace:
                return std::holds_alternative<OperationalSpaceTuning>(tuning);
            case ArmControllerKind::slotineLi:
                return std::holds_alternative<SlotineLiTuning>(tuning);
            default:
                return false;
        }
    }

    std::optional<TuningViolation> CheckTuning(const ArmControllerTuning& tuning)
    {
        return std::visit(Checker{}, tuning);
    }

    ArmControllerTuning TuningOf(const ArmConfig& config, ArmControllerKind kind)
    {
        switch (kind)
        {
            case ArmControllerKind::computedTorque:
                return config.computedTorque;
            case ArmControllerKind::gravityPid:
                return config.gravityPid;
            case ArmControllerKind::jointImpedance:
                return config.jointImpedance;
            case ArmControllerKind::cartesianImpedance:
                return config.cartesianImpedance;
            case ArmControllerKind::operationalSpace:
                return config.operationalSpace;
            case ArmControllerKind::slotineLi:
                return config.slotineLi;
            default:
                return config.gravityPd;
        }
    }

    void StoreTuning(ArmConfig& config, ArmControllerKind kind, const ArmControllerTuning& tuning)
    {
        if (!TuningFits(kind, tuning))
            return;
        switch (kind)
        {
            case ArmControllerKind::gravityPd:
                config.gravityPd = std::get<BandwidthTuning>(tuning);
                break;
            case ArmControllerKind::computedTorque:
                config.computedTorque = std::get<BandwidthTuning>(tuning);
                break;
            case ArmControllerKind::gravityPid:
                config.gravityPid = std::get<BandwidthTuning>(tuning);
                break;
            case ArmControllerKind::jointImpedance:
                config.jointImpedance = std::get<JointImpedanceTuning>(tuning);
                break;
            case ArmControllerKind::cartesianImpedance:
                config.cartesianImpedance = std::get<CartesianImpedanceTuning>(tuning);
                break;
            case ArmControllerKind::operationalSpace:
                config.operationalSpace = std::get<OperationalSpaceTuning>(tuning);
                break;
            case ArmControllerKind::slotineLi:
                config.slotineLi = std::get<SlotineLiTuning>(tuning);
                break;
            default:
                break;
        }
    }

    PresetResult SavePreset(ArmConfig& config, ArmControllerKind kind, infra::BoundedConstString name)
    {
        if (!Tunable(kind))
            return PresetResult::notTunable;
        if (name.empty() || name.size() > presetNameSize)
            return PresetResult::badName;

        const auto begin = config.presets.begin();
        const auto end = begin + config.presetCount;
        auto found = std::find_if(begin, end, [kind, name](const ArmControllerPreset& preset)
            {
                return preset.controller == kind && preset.name == name;
            });
        const auto replaced = found != end;
        if (!replaced)
        {
            if (config.presetCount == maxArmControllerPresets)
                return PresetResult::full;
            ++config.presetCount;
        }
        found->name = name;
        found->controller = kind;
        found->tuning = TuningOf(config, kind);
        return replaced ? PresetResult::replaced : PresetResult::saved;
    }
}
