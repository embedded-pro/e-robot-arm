#include "tools/tuning/TuningMetrics.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/StepResponseMetrics.hpp"
#include <algorithm>
#include <memory>

namespace tuning
{
    namespace
    {
        constexpr float riseLevel{ 0.9f };
        constexpr float settlingBand{ 0.02f };

        using Response = math::Vector<float, maxSamples>;

        std::unique_ptr<Response> Normalised(const std::vector<TuningSample>& samples, float start, float amplitude)
        {
            auto response = std::make_unique<Response>();
            const auto count = std::min(samples.size(), maxSamples);
            for (std::size_t index = 0; index != maxSamples; ++index)
            {
                const auto& sample = samples[std::min(index, count - 1)];
                response->at(index, 0) = (sample.measured - start) / amplitude;
            }
            return response;
        }

        float Period(const std::vector<TuningSample>& samples)
        {
            if (samples.size() < 2)
                return 0.0f;
            return (samples.back().time - samples.front().time) / static_cast<float>(samples.size() - 1);
        }
    }

    StepMetrics MeasureStep(const std::vector<TuningSample>& samples, float start, float amplitude)
    {
        StepMetrics metrics;
        if (samples.empty() || amplitude == 0.0f)
            return metrics;

        const auto response = Normalised(samples, start, amplitude);
        const auto dt = Period(samples);
        const auto last = response->at(maxSamples - 1, 0);
        float peak{ response->at(0, 0) };
        for (std::size_t index = 0; index != maxSamples; ++index)
            peak = std::max(peak, response->at(index, 0));

        if (peak >= riseLevel)
            metrics.riseTime = math::RiseTime(*response, 1.0f, dt);
        metrics.overshootPercent = std::max(0.0f, math::PercentOvershoot(*response, 1.0f));
        if (math::Abs(last - 1.0f) <= settlingBand)
            metrics.settlingTime = math::SettlingTime(*response, 1.0f, settlingBand, dt);
        metrics.steadyStateError = math::SteadyStateError(*response, 1.0f) * math::Abs(amplitude);
        return metrics;
    }

    TrackingMetrics MeasureTracking(const std::vector<TuningSample>& samples)
    {
        TrackingMetrics metrics;
        if (samples.empty())
            return metrics;

        float sum{ 0.0f };
        for (const auto& sample : samples)
        {
            const auto error = sample.commanded - sample.measured;
            sum += error * error;
            metrics.peak = std::max(metrics.peak, math::Abs(error));
        }
        metrics.rms = math::Sqrt(sum / static_cast<float>(samples.size()));
        return metrics;
    }

    void WriteCsv(const std::vector<TuningSample>& samples, std::ostream& output)
    {
        output << "time_s,commanded_rad,measured_rad,tracking_error_rad,torque_nm,dropped_before\n";
        for (const auto& sample : samples)
            output << sample.time << ',' << sample.commanded << ',' << sample.measured << ',' << sample.commanded - sample.measured << ',' << sample.torque << ','
                   << sample.droppedBefore << '\n';
    }
}
