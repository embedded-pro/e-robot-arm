#ifndef TOOLS_ARM_HOST_TEST_ARM_HOST_TEST_SUPPORT_HPP
#define TOOLS_ARM_HOST_TEST_ARM_HOST_TEST_SUPPORT_HPP

#include "core/arm/Arm.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include <algorithm>
#include <chrono>
#include <numbers>
#include <vector>

namespace arm_host::test
{
    inline constexpr arm::JointNodeIds nodes{ 21, 22, 23, 24, 25, 26 };

    inline arm::ArmConfig Config()
    {
        arm::ArmConfig config{ nodes, 3, {}, {}, std::chrono::milliseconds{ 10 } };
        for (auto& limit : config.positionLimits)
            limit = { foc::Radians{ -std::numbers::pi_v<float> }, foc::Radians{ std::numbers::pi_v<float> } };
        for (auto& limit : config.motionLimits)
            limit = { foc::RadiansPerSecond{ 1.0f }, foc::RadiansPerSecondSquared{ 2.0f } };
        return config;
    }

    class RecordingSink
        : public ArmSessionSink
    {
    public:
        void OnEvent(const ArmEvent& event) override
        {
            events.push_back(event);
        }

        void OnSnapshot(const ArmSnapshot& newSnapshot) override
        {
            snapshot = newSnapshot;
            ++snapshots;
        }

        void OnTrace(const TraceSample& sample) override
        {
            trace.push_back(sample);
        }

        const ArmEvent* Find(ArmEventKind kind) const
        {
            const auto found = std::ranges::find(events, kind, &ArmEvent::kind);
            return found == events.end() ? nullptr : &*found;
        }

        const ArmEvent* Last() const
        {
            return events.empty() ? nullptr : &events.back();
        }

        std::vector<ArmEvent> events;
        ArmSnapshot snapshot;
        uint32_t snapshots{ 0 };
        std::vector<TraceSample> trace;
    };
}

#endif
