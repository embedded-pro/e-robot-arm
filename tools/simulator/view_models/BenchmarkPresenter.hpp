#pragma once

#include "tools/arm_host/ArmCommands.hpp"
#include "tools/arm_host/ArmEvent.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/simulator/view_models/ControlRules.hpp"
#include "tools/tuning/Benchmark.hpp"
#include "ui/charts/Series.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace simulator
{
    struct BenchmarkChoice
    {
        arm::ArmControllerKind kind{ arm::ArmControllerKind::gravityPd };
        std::string name;
        bool chosen{ false };

        bool operator==(const BenchmarkChoice& other) const = default;
    };

    inline constexpr std::size_t benchmarkColumnCount{ 5 };

    struct BenchmarkRow
    {
        std::string controller;
        std::array<std::string, benchmarkColumnCount> values{};
        std::string result;
        ui::theme::StatusLevel level{ ui::theme::StatusLevel::Neutral };

        bool operator==(const BenchmarkRow& other) const = default;
    };

    struct BenchmarkChart
    {
        std::vector<float> time;
        std::vector<ui::charts::ChartPanel> panels;
    };

    class BenchmarkPresenter
    {
    public:
        static constexpr std::array<std::string_view, benchmarkColumnCount> columns{ "RMS err", "Peak err", "Peak τ", "Energy", "Settle" };

        explicit BenchmarkPresenter(arm_host::ArmCommands& arm);
        BenchmarkPresenter(const BenchmarkPresenter& other) = delete;
        BenchmarkPresenter& operator=(const BenchmarkPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events, std::span<const arm_host::TraceSample> trace);
        void Choose(std::size_t choice, bool chosen);
        void Run();
        void Abort();

        const std::vector<BenchmarkChoice>& Choices() const;
        ControlState RunState() const;
        ControlState AbortState() const;
        const std::string& Status() const;
        ui::theme::StatusLevel StatusLevel() const;
        const std::vector<BenchmarkRow>& Rows() const;
        const BenchmarkChart& Chart() const;
        uint32_t ChartRevision() const;
        bool CanExport() const;
        void WriteCsv(std::ostream& output) const;

    private:
        std::vector<arm::ArmControllerKind> Chosen() const;
        bool Running() const;
        void UpdateChoices();
        void UpdateStatus();
        void UpdateRows();
        void UpdateChart();

        arm_host::ArmCommands& arm;
        arm_host::ArmSnapshot snapshot;
        std::vector<BenchmarkChoice> choices;
        std::optional<tuning::Benchmark> benchmark;
        std::optional<std::string> startRefusal;
        std::string status{ "Choose controllers, then Run" };
        ui::theme::StatusLevel statusLevel{ ui::theme::StatusLevel::Neutral };
        std::vector<BenchmarkRow> rows;
        BenchmarkChart chart;
        uint32_t chartRevision{ 0 };
        std::size_t shownResults{ 0 };
        tuning::BenchmarkState shownState{ tuning::BenchmarkState::idle };
    };
}
