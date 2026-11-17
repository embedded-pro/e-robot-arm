#include "tools/simulator/view_models/TuningPresenter.hpp"
#include <algorithm>
#include <format>

namespace simulator
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::FieldKind;
        using ui::model::OptionSpec;
        using ui::theme::ButtonRole;

        constexpr std::array<OptionSpec, arm::jointCount> jointOptions{ { { "J1" }, { "J2" }, { "J3" }, { "J4" }, { "J5" }, { "J6" } } };
        constexpr std::array<OptionSpec, 2> testOptions{ { { "Step" }, { "Out and Back" } } };

        constexpr std::array<ui::model::FieldSpec, 3> fields{ {
            { TuningField::joint, ui::model::noGroup, FieldKind::Choice, "Joint", "", {}, jointOptions, {}, {} },
            { TuningField::test, ui::model::noGroup, FieldKind::Choice, "Test", "", {}, testOptions, {}, {} },
            { TuningField::amplitude, ui::model::noGroup, FieldKind::Number, "Amplitude", "rad", { -3.0, 3.0, 0.01, 0.05, 3 }, {}, {}, {} },
        } };

        constexpr std::array<ActionSpec, tuningActionCount> actions{ {
            { TuningAction::run, "Run", ButtonRole::Default },
            { TuningAction::keepReference, "Keep as Reference" },
            { TuningAction::clearReference, "Clear Reference", ButtonRole::Reset },
        } };

        constexpr std::array<std::string_view, tuningMetricCount> metricNames{ "Rise time", "Overshoot", "Settling time", "Steady-state error",
            "RMS tracking error", "Peak tracking error" };

        std::string_view KindName(tuning::TestKind kind)
        {
            return kind == tuning::TestKind::step ? "step" : "out and back";
        }

        std::string Seconds(const std::optional<float>& value)
        {
            return value.has_value() ? std::format("{:.3f} s", *value) : std::string{ "—" };
        }

        std::string Radians(float value)
        {
            return std::format("{:.4f} rad", std::abs(value) < 5.0e-5f ? 0.0f : value);
        }
    }

    TuningPresenter::Form::Form()
        : spec{ {}, fields, actions, {} }
        , model(spec, values, {})
    {
        model.ResetToDefaults();
    }

    TuningPresenter::TuningPresenter(ui::shell::FormView& view, arm_host::ArmCommands& arm)
        : view(view)
        , arm(arm)
    {
        form.model.onActionTriggered = [this](ui::model::ActionId action)
        {
            OnAction(action);
        };
        form.model.onFieldChanged = [this](ui::model::FieldId)
        {
            Update();
        };
        view.Build(form.model);
        UpdateMetrics();
        Update();
    }

    void TuningPresenter::Refresh(const arm_host::ArmSnapshot& newSnapshot, std::span<const arm_host::ArmEvent> events,
        std::span<const arm_host::TraceSample> trace)
    {
        const bool switched = !(newSnapshot.target == snapshot.target);
        snapshot = newSnapshot;
        if (switched)
        {
            run.reset();
            last.reset();
            reference.reset();
            lastIsReference = false;
            status.clear();
            statusLevel = ui::theme::StatusLevel::Neutral;
            UpdateMetrics();
            UpdateChart();
        }

        Follow(events, trace);
        Update();
    }

    void TuningPresenter::SelectJoint(std::size_t joint)
    {
        if (joint >= arm::jointCount || run.has_value())
            return;

        form.model.SetSelection(TuningField::joint, joint);
        view.Refresh(TuningField::joint);
        Update();
    }

    bool TuningPresenter::Enabled(ui::model::ActionId action) const
    {
        return refusals[action.value].empty();
    }

    bool TuningPresenter::CanExport() const
    {
        return last.has_value();
    }

    void TuningPresenter::WriteCsv(std::ostream& output) const
    {
        if (last.has_value())
            tuning::WriteCsv(last->samples, output);
    }

    const std::string& TuningPresenter::Hint() const
    {
        return hint;
    }

    const std::string& TuningPresenter::Status() const
    {
        return status;
    }

    ui::theme::StatusLevel TuningPresenter::StatusLevel() const
    {
        return statusLevel;
    }

    const std::array<TuningMetricRow, tuningMetricCount>& TuningPresenter::Metrics() const
    {
        return metrics;
    }

    const TuningChart& TuningPresenter::Chart() const
    {
        return chart;
    }

    uint32_t TuningPresenter::ChartRevision() const
    {
        return chartRevision;
    }

    void TuningPresenter::OnAction(ui::model::ActionId action)
    {
        Update();
        if (!Enabled(action))
            return;

        if (action == TuningAction::run)
            Run();
        else if (action == TuningAction::keepReference)
        {
            reference = last;
            lastIsReference = true;
        }
        else
        {
            reference.reset();
            lastIsReference = false;
        }

        UpdateMetrics();
        UpdateChart();
        Update();
    }

    void TuningPresenter::Run()
    {
        const auto test = Test();
        runLabel = Label(test);
        run.emplace(arm, test);
        if (const auto refusal = run->Start())
        {
            status = "Failed: " + *refusal;
            statusLevel = ui::theme::StatusLevel::Fault;
            run.reset();
            return;
        }

        status = std::format("Requested {} on J{}…", KindName(test.kind), test.joint + 1);
        statusLevel = ui::theme::StatusLevel::Neutral;
    }

    void TuningPresenter::Follow(std::span<const arm_host::ArmEvent> events, std::span<const arm_host::TraceSample> trace)
    {
        if (!run.has_value())
            return;

        for (const auto& event : events)
            run->OnEvent(event);
        for (const auto& sample : trace)
            run->OnTrace(sample);

        const auto state = run->State();
        if (state == tuning::RunState::done || state == tuning::RunState::failed)
            Finish();
        else if (state != tuning::RunState::starting)
            status = std::format("Running {} on J{}…", KindName(run->Test().kind), run->Test().joint + 1);
    }

    void TuningPresenter::Finish()
    {
        if (run->State() == tuning::RunState::failed)
        {
            status = "Failed: " + run->Failure();
            statusLevel = ui::theme::StatusLevel::Fault;
        }
        else
        {
            last = Result{ runLabel, run->Test(), run->Samples(), run->Step(), run->Tracking() };
            lastIsReference = false;
            status = "Done: " + runLabel;
            statusLevel = ui::theme::StatusLevel::Ok;
            UpdateMetrics();
            UpdateChart();
        }
        run.reset();
    }

    tuning::TuningTest TuningPresenter::Test() const
    {
        return tuning::TuningTest{ .kind = form.model.Selection(TuningField::test) == 0 ? tuning::TestKind::step : tuning::TestKind::outAndBack,
            .joint = form.model.Selection(TuningField::joint),
            .amplitude = static_cast<float>(form.model.Number(TuningField::amplitude)) };
    }

    std::string TuningPresenter::Label(const tuning::TuningTest& test) const
    {
        const auto active = std::ranges::find_if(snapshot.armControllers, &arm_host::ArmControllerStatus::active);
        const auto controller = active != snapshot.armControllers.end() ? active->name : std::string{ "no controller" };
        return std::format("{} · {} {:.3f} rad · J{}", controller, KindName(test.kind), test.amplitude, test.joint + 1);
    }

    std::string TuningPresenter::Refusal(ui::model::ActionId action) const
    {
        if (action == TuningAction::run)
        {
            if (run.has_value())
                return "a test is running";
            return tuning::Refusal(Test(), snapshot).value_or(std::string{});
        }
        if (action == TuningAction::keepReference)
        {
            if (!last.has_value())
                return "no finished run";
            return lastIsReference ? "the last run is the reference" : std::string{};
        }
        return reference.has_value() ? std::string{} : "no reference";
    }

    void TuningPresenter::Update()
    {
        hint.clear();
        for (std::size_t index = 0; index != tuningActionCount; ++index)
        {
            const auto action = actions[index].id;
            refusals[index] = Refusal(action);
            view.SetActionEnabled(action, refusals[index].empty());
            if (!refusals[index].empty())
                hint += std::format("{}{}: {}", hint.empty() ? "" : "\n", actions[index].label, refusals[index]);
        }
    }

    void TuningPresenter::UpdateMetrics()
    {
        const auto cells = [](const std::optional<Result>& result)
        {
            std::array<std::string, tuningMetricCount> values;
            values.fill("—");
            if (!result.has_value())
                return values;

            if (result->step.has_value())
            {
                values[0] = Seconds(result->step->riseTime);
                values[1] = std::format("{:.1f} %", result->step->overshootPercent);
                values[2] = Seconds(result->step->settlingTime);
                values[3] = Radians(result->step->steadyStateError);
            }
            values[4] = Radians(result->tracking.rms);
            values[5] = Radians(result->tracking.peak);
            return values;
        };

        const auto current = cells(last);
        const auto kept = cells(reference);
        for (std::size_t index = 0; index != tuningMetricCount; ++index)
            metrics[index] = { metricNames[index], current[index], kept[index] };
    }

    void TuningPresenter::UpdateChart()
    {
        ++chartRevision;
        chart.time.clear();
        chart.panels.clear();
        if (!last.has_value() && !reference.has_value())
            return;

        const auto& longest = !reference.has_value() || (last.has_value() && last->samples.size() >= reference->samples.size()) ? *last : *reference;
        for (const auto& sample : longest.samples)
            chart.time.push_back(sample.time);

        const auto& theme = ui::theme::Current();
        const auto column = [](const std::optional<Result>& result, auto member)
        {
            std::vector<float> values;
            if (result.has_value())
                for (const auto& sample : result->samples)
                    values.push_back(member(sample));
            return values;
        };
        const auto commanded = [](const tuning::TuningSample& sample)
        {
            return sample.commanded;
        };
        const auto measured = [](const tuning::TuningSample& sample)
        {
            return sample.measured;
        };
        const auto error = [](const tuning::TuningSample& sample)
        {
            return sample.commanded - sample.measured;
        };
        const auto torque = [](const tuning::TuningSample& sample)
        {
            return sample.torque;
        };

        ui::charts::ChartPanel position{ "Position", "rad", {}, 2 };
        ui::charts::ChartPanel tracking{ "Tracking error", "rad", {}, 1 };
        ui::charts::ChartPanel effort{ "Torque", "N·m", {}, 1 };
        if (last.has_value())
        {
            position.series.push_back({ "Cmd", theme.Series(0), column(last, commanded) });
            position.series.push_back({ "Meas", theme.Series(1), column(last, measured) });
            tracking.series.push_back({ "Err", theme.Series(2), column(last, error) });
            effort.series.push_back({ "Torque", theme.Series(3), column(last, torque) });
        }
        if (reference.has_value())
        {
            position.series.push_back({ "Ref meas", theme.Series(4), column(reference, measured) });
            tracking.series.push_back({ "Ref err", theme.Series(5), column(reference, error) });
            effort.series.push_back({ "Ref torque", theme.Series(6), column(reference, torque) });
        }
        chart.panels = { std::move(position), std::move(tracking), std::move(effort) };
    }
}
