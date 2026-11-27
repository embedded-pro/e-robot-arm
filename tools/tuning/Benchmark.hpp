#pragma once

#include "core/arm/ArmConfig.hpp"
#include "tools/arm_host/ArmCommands.hpp"
#include "tools/arm_host/ArmEvent.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace tuning
{
    inline constexpr std::size_t maxBenchmarkControllers{ 8 };
    inline constexpr std::size_t maxBenchmarkSamples{ 6000 };
    inline constexpr float benchmarkHoldSeconds{ 1.0f };
    inline constexpr float benchmarkSettleBand{ 0.005f };

    struct BenchmarkSample
    {
        float time{ 0.0f };
        float errorNorm{ 0.0f };
    };

    struct BenchmarkMetrics
    {
        float rmsError{ 0.0f };
        float peakError{ 0.0f };
        float peakTorque{ 0.0f };
        float energy{ 0.0f };
        std::optional<float> settleTime;
    };

    enum class BenchmarkRunState : uint8_t
    {
        pending,
        running,
        done,
        failed
    };

    struct BenchmarkResult
    {
        arm::ArmControllerKind controller{ arm::ArmControllerKind::gravityPd };
        arm::ArmControllerTuning tuning{ arm::BandwidthTuning{} };
        BenchmarkRunState state{ BenchmarkRunState::pending };
        std::string failure;
        BenchmarkMetrics metrics;
        std::vector<BenchmarkSample> samples;
    };

    enum class BenchmarkState : uint8_t
    {
        idle,
        running,
        done,
        failed,
        aborted
    };

    std::vector<arm::JointPositions> BenchmarkPath(const arm::ArmConfig& config, const arm::JointPositions& start);
    std::optional<std::string> BenchmarkRefusal(const std::vector<arm::ArmControllerKind>& controllers, const arm_host::ArmSnapshot& snapshot);

    void WriteBenchmarkCsv(const std::vector<BenchmarkResult>& results, std::ostream& output);

    class Benchmark
    {
    public:
        Benchmark(arm_host::ArmCommands& arm, std::vector<arm::ArmControllerKind> controllers);

        std::optional<std::string> Start();
        void Abort();
        void OnEvent(const arm_host::ArmEvent& event);
        void OnTrace(const arm_host::TraceSample& sample);

        BenchmarkState State() const;
        std::size_t Current() const;
        const std::vector<BenchmarkResult>& Results() const;
        const std::string& Failure() const;

    private:
        enum class Step : uint8_t
        {
            selecting,
            holding,
            moving,
            recording
        };

        void Begin(std::size_t run);
        void NextLeg();
        void Finish();
        void Fail(std::string reason, bool stopMove = false);
        void Restore();
        void Record(const arm_host::TraceSample& sample, float time);
        float Accumulate(const arm_host::TraceSample& sample, BenchmarkMetrics& metrics);
        std::optional<arm::ArmControllerId> IdOf(arm::ArmControllerKind kind) const;

        arm_host::ArmCommands& commands;
        std::vector<arm::ArmControllerKind> controllers;
        arm_host::ArmSnapshot snapshot;
        std::vector<arm::JointPositions> path;
        std::vector<BenchmarkResult> results;
        std::optional<arm::ArmControllerId> original;
        BenchmarkState state{ BenchmarkState::idle };
        std::string failure;
        Step step{ Step::selecting };
        std::size_t current{ 0 };
        std::size_t leg{ 0 };
        uint32_t pending{ 0 };
        std::optional<infra::TimePoint> since;
        float lastMoveDone{ 0.0f };
        float lastOutside{ 0.0f };
        std::optional<arm::JointPositions> previousMeasured;
        float squaredSum{ 0.0f };
    };
}
