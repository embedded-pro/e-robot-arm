#pragma once

#include "tools/arm_host/ArmCommands.hpp"
#include "tools/arm_host/ArmEvent.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/tuning/TuningMetrics.hpp"
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace tuning
{
    enum class TestKind : uint8_t
    {
        step,
        outAndBack
    };

    struct TuningTest
    {
        TestKind kind{ TestKind::step };
        std::size_t joint{ 0 };
        float amplitude{ 0.1f };
        std::chrono::milliseconds record{ 2000 };
    };

    enum class RunState : uint8_t
    {
        idle,
        starting,
        recording,
        returning,
        settling,
        done,
        failed
    };

    std::optional<std::string> Refusal(const TuningTest& test, const arm_host::ArmSnapshot& snapshot);

    class TuningRun
    {
    public:
        static constexpr float settleSeconds{ 0.5f };

        TuningRun(arm_host::ArmCommands& arm, const TuningTest& test);

        std::optional<std::string> Start();
        void OnEvent(const arm_host::ArmEvent& event);
        void OnTrace(const arm_host::TraceSample& sample);

        uint32_t Command() const;
        RunState State() const;
        const std::string& Failure() const;
        const TuningTest& Test() const;
        const std::vector<TuningSample>& Samples() const;
        std::optional<StepMetrics> Step() const;
        TrackingMetrics Tracking() const;
        uint64_t DroppedSamples() const;

    private:
        void OnAnswer(const arm_host::ArmEvent& event);
        void OnMoveDone();
        void Record(const arm_host::TraceSample& sample);
        void Fail(std::string reason);

        arm_host::ArmCommands& arm;
        TuningTest test;
        RunState state{ RunState::idle };
        std::string failure;
        arm::JointPositions origin{};
        uint32_t pending{ 0 };
        bool back{ false };
        std::optional<infra::TimePoint> firstTime;
        float settleStart{ 0.0f };
        std::vector<TuningSample> samples;
        uint64_t droppedSamples{ 0 };
    };
}
