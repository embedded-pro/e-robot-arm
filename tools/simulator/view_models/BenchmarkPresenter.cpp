#include "tools/simulator/view_models/BenchmarkPresenter.hpp"
#include <algorithm>
#include <cctype>
#include <format>

namespace simulator
{
    namespace
    {
        using tuning::BenchmarkRunState;
        using tuning::BenchmarkState;

        constexpr std::array<std::string_view, benchmarkColumnCount> units{ "rad", "rad", "N·m", "J", "s" };
        constexpr std::array<int, benchmarkColumnCount> decimals{ 4, 4, 2, 2, 2 };

        std::optional<float> Value(const tuning::BenchmarkMetrics& metrics, std::size_t column)
        {
            switch (column)
            {
                case 0:
                    return metrics.rmsError;
                case 1:
                    return metrics.peakError;
                case 2:
                    return metrics.peakTorque;
                case 3:
                    return metrics.energy;
                default:
                    return metrics.settleTime;
            }
        }

        std::string Result(const tuning::BenchmarkResult& result)
        {
            switch (result.state)
            {
                case BenchmarkRunState::done:
                    return "Done";
                case BenchmarkRunState::running:
                    return "Running…";
                case BenchmarkRunState::failed:
                    return "Failed: " + result.failure;
                default:
                    return "Waiting";
            }
        }

        ui::theme::StatusLevel Level(BenchmarkRunState state)
        {
            if (state == BenchmarkRunState::done)
                return ui::theme::StatusLevel::Ok;
            return state == BenchmarkRunState::failed ? ui::theme::StatusLevel::Fault : ui::theme::StatusLevel::Neutral;
        }

        std::array<std::optional<float>, benchmarkColumnCount> Best(const std::vector<tuning::BenchmarkResult>& results)
        {
            std::array<std::optional<float>, benchmarkColumnCount> best{};
            for (const auto& result : results)
                for (std::size_t column = 0; result.state == BenchmarkRunState::done && column != benchmarkColumnCount; ++column)
                    if (const auto value = Value(result.metrics, column); value.has_value() && (!best[column].has_value() || *value < *best[column]))
                        best[column] = value;
            return best;
        }
    }

    BenchmarkPresenter::BenchmarkPresenter(arm_host::ArmCommands& arm)
        : arm(arm)
    {}

    void BenchmarkPresenter::Refresh(const arm_host::ArmSnapshot& newSnapshot, std::span<const arm_host::ArmEvent> events, std::span<const arm_host::TraceSample> trace)
    {
        snapshot = newSnapshot;
        UpdateChoices();
        if (benchmark.has_value())
        {
            for (const auto& event : events)
                benchmark->OnEvent(event);
            for (const auto& sample : trace)
                benchmark->OnTrace(sample);
        }
        UpdateStatus();
        UpdateRows();
        const auto results = benchmark.has_value() ? benchmark->Results().size() : 0;
        const auto state = benchmark.has_value() ? benchmark->State() : BenchmarkState::idle;
        if (results != shownResults || state != shownState || (Running() && !trace.empty()))
        {
            shownResults = results;
            shownState = state;
            UpdateChart();
        }
    }

    void BenchmarkPresenter::Choose(std::size_t choice, bool chosen)
    {
        if (choice < choices.size() && !Running())
            choices[choice].chosen = chosen;
    }

    void BenchmarkPresenter::Run()
    {
        if (!RunState().enabled)
            return;
        benchmark.emplace(arm, Chosen());
        startRefusal = benchmark->Start();
        if (startRefusal.has_value())
            benchmark.reset();
        UpdateStatus();
        UpdateRows();
        UpdateChart();
    }

    void BenchmarkPresenter::Abort()
    {
        if (Running())
            benchmark->Abort();
        UpdateStatus();
        UpdateRows();
    }

    const std::vector<BenchmarkChoice>& BenchmarkPresenter::Choices() const
    {
        return choices;
    }

    ControlState BenchmarkPresenter::RunState() const
    {
        if (Running())
            return { false, "Running: abort first" };
        if (const auto refusal = tuning::BenchmarkRefusal(Chosen(), snapshot))
        {
            auto text = *refusal;
            if (!text.empty())
                text.front() = static_cast<char>(std::toupper(static_cast<unsigned char>(text.front())));
            return { false, text };
        }
        if (arm_host::FreeMode(snapshot))
            return { false, "Free mode: select a tracking controller" };
        return { true, {} };
    }

    ControlState BenchmarkPresenter::AbortState() const
    {
        return Running() ? ControlState{ true, {} } : ControlState{ false, "No benchmark running" };
    }

    const std::string& BenchmarkPresenter::Status() const
    {
        return status;
    }

    ui::theme::StatusLevel BenchmarkPresenter::StatusLevel() const
    {
        return statusLevel;
    }

    const std::vector<BenchmarkRow>& BenchmarkPresenter::Rows() const
    {
        return rows;
    }

    const BenchmarkChart& BenchmarkPresenter::Chart() const
    {
        return chart;
    }

    uint32_t BenchmarkPresenter::ChartRevision() const
    {
        return chartRevision;
    }

    bool BenchmarkPresenter::CanExport() const
    {
        return benchmark.has_value() && !Running();
    }

    void BenchmarkPresenter::WriteCsv(std::ostream& output) const
    {
        if (CanExport())
            tuning::WriteBenchmarkCsv(benchmark->Results(), output);
    }

    std::vector<arm::ArmControllerKind> BenchmarkPresenter::Chosen() const
    {
        std::vector<arm::ArmControllerKind> chosen;
        for (const auto& choice : choices)
            if (choice.chosen)
                chosen.push_back(choice.kind);
        return chosen;
    }

    bool BenchmarkPresenter::Running() const
    {
        return benchmark.has_value() && benchmark->State() == BenchmarkState::running;
    }

    void BenchmarkPresenter::UpdateChoices()
    {
        std::vector<BenchmarkChoice> offered;
        for (const auto& controller : snapshot.armControllers)
        {
            if (!controller.enabled || controller.passive)
                continue;
            for (std::size_t kind = 0; kind != arm::armControllerNames.size(); ++kind)
                if (controller.name == arm::armControllerNames[kind])
                {
                    const auto previous = std::ranges::find(choices, controller.name, &BenchmarkChoice::name);
                    offered.push_back({ static_cast<arm::ArmControllerKind>(kind), controller.name, previous != choices.end() && previous->chosen });
                }
        }
        choices = std::move(offered);
    }

    void BenchmarkPresenter::UpdateStatus()
    {
        using enum ui::theme::StatusLevel;
        if (startRefusal.has_value())
        {
            status = "Not started: " + *startRefusal;
            statusLevel = Fault;
            return;
        }
        if (!benchmark.has_value())
            return;
        const auto& results = benchmark->Results();
        switch (benchmark->State())
        {
            case BenchmarkState::running:
                status = std::format("Running {} of {}: {}…", benchmark->Current() + 1, results.size(), arm::ArmControllerName(results[benchmark->Current()].controller));
                statusLevel = Neutral;
                break;
            case BenchmarkState::done:
                status = std::format("Done: {} controllers", results.size());
                statusLevel = Ok;
                break;
            case BenchmarkState::aborted:
                status = "Aborted";
                statusLevel = Warning;
                break;
            default:
                status = "Failed: " + benchmark->Failure();
                statusLevel = Fault;
                break;
        }
    }

    void BenchmarkPresenter::UpdateRows()
    {
        rows.clear();
        if (!benchmark.has_value())
            return;
        const auto& results = benchmark->Results();
        const auto best = Best(results);
        for (const auto& result : results)
        {
            BenchmarkRow row{ arm::ArmControllerName(result.controller), {}, Result(result), Level(result.state) };
            for (std::size_t column = 0; column != benchmarkColumnCount; ++column)
            {
                const auto value = result.state == BenchmarkRunState::done ? Value(result.metrics, column) : std::nullopt;
                if (!value.has_value())
                {
                    row.values[column] = "—";
                    continue;
                }
                row.values[column] = std::format("{:.{}f} {}", *value, decimals[column], units[column]);
                if (results.size() > 1 && best[column] == value)
                    row.values[column] += " ▲";
            }
            rows.push_back(std::move(row));
        }
    }

    void BenchmarkPresenter::UpdateChart()
    {
        ++chartRevision;
        chart.time.clear();
        chart.panels.clear();
        if (!benchmark.has_value())
            return;

        const auto& results = benchmark->Results();
        const auto longest = std::ranges::max_element(results, {}, [](const tuning::BenchmarkResult& result)
            {
                return result.samples.size();
            });
        if (longest == results.end() || longest->samples.empty())
            return;
        for (const auto& sample : longest->samples)
            chart.time.push_back(sample.time);

        ui::charts::ChartPanel panel{ "Tracking error", "rad", {}, 1 };
        const auto& theme = ui::theme::Current();
        for (std::size_t index = 0; index != results.size(); ++index)
        {
            std::vector<float> values;
            for (const auto& sample : results[index].samples)
                values.push_back(sample.errorNorm);
            panel.series.emplace_back(std::string{ arm::ArmControllerName(results[index].controller) }, theme.Series(index), std::move(values));
        }
        chart.panels.push_back(std::move(panel));
    }
}
