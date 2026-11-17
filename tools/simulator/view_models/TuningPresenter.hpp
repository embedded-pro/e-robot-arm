#ifndef TOOLS_SIMULATOR_VIEW_MODELS_TUNING_PRESENTER_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_TUNING_PRESENTER_HPP

#include "tools/arm_host/ArmCommands.hpp"
#include "tools/arm_host/ArmEvent.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/tuning/TuningRun.hpp"
#include "ui/charts/Series.hpp"
#include "ui/model/FormModel.hpp"
#include "ui/shell/FormView.hpp"
#include "ui/theme/Theme.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <ostream>
#include <span>
#include <string>
#include <vector>

namespace simulator
{
    struct TuningField
    {
        static constexpr ui::model::FieldId joint{ 0 };
        static constexpr ui::model::FieldId test{ 1 };
        static constexpr ui::model::FieldId amplitude{ 2 };
    };

    struct TuningAction
    {
        static constexpr ui::model::ActionId run{ 0 };
        static constexpr ui::model::ActionId keepReference{ 1 };
        static constexpr ui::model::ActionId clearReference{ 2 };
    };

    inline constexpr std::size_t tuningActionCount{ 3 };
    inline constexpr std::size_t tuningMetricCount{ 6 };

    struct TuningMetricRow
    {
        std::string_view name;
        std::string current;
        std::string reference;
    };

    struct TuningChart
    {
        std::vector<float> time;
        std::vector<ui::charts::ChartPanel> panels;
    };

    class TuningPresenter
    {
    public:
        TuningPresenter(ui::shell::FormView& view, arm_host::ArmCommands& arm);
        TuningPresenter(const TuningPresenter& other) = delete;
        TuningPresenter& operator=(const TuningPresenter& other) = delete;

        void Refresh(const arm_host::ArmSnapshot& snapshot, std::span<const arm_host::ArmEvent> events, std::span<const arm_host::TraceSample> trace);
        void SelectJoint(std::size_t joint);

        bool Enabled(ui::model::ActionId action) const;
        bool CanExport() const;
        void WriteCsv(std::ostream& output) const;
        const std::string& Hint() const;
        const std::string& Status() const;
        ui::theme::StatusLevel StatusLevel() const;
        const std::array<TuningMetricRow, tuningMetricCount>& Metrics() const;
        const TuningChart& Chart() const;
        uint32_t ChartRevision() const;

    private:
        struct Result
        {
            std::string label;
            tuning::TuningTest test;
            std::vector<tuning::TuningSample> samples;
            std::optional<tuning::StepMetrics> step;
            tuning::TrackingMetrics tracking;
        };

        struct Form
        {
            Form();
            Form(const Form& other) = delete;
            Form& operator=(const Form& other) = delete;

            ui::model::FormSpec spec;
            std::array<ui::model::FieldValue, 3> values{};
            ui::model::FormModel model;
        };

        void OnAction(ui::model::ActionId action);
        void Run();
        void Follow(std::span<const arm_host::ArmEvent> events, std::span<const arm_host::TraceSample> trace);
        void Finish();
        tuning::TuningTest Test() const;
        std::string Label(const tuning::TuningTest& test) const;
        std::string Refusal(ui::model::ActionId action) const;
        void Update();
        void UpdateMetrics();
        void UpdateChart();

        ui::shell::FormView& view;
        arm_host::ArmCommands& arm;
        Form form;
        arm_host::ArmSnapshot snapshot;
        std::optional<tuning::TuningRun> run;
        std::string runLabel;
        std::optional<Result> last;
        std::optional<Result> reference;
        bool lastIsReference{ false };
        std::array<std::string, tuningActionCount> refusals{};
        std::string hint;
        std::string status;
        ui::theme::StatusLevel statusLevel{ ui::theme::StatusLevel::Neutral };
        std::array<TuningMetricRow, tuningMetricCount> metrics{};
        TuningChart chart;
        uint32_t chartRevision{ 0 };
    };
}

#endif
