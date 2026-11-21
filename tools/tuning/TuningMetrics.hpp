#pragma once

#include <cstddef>
#include <optional>
#include <ostream>
#include <vector>

namespace tuning
{
    inline constexpr std::size_t maxSamples{ 2000 };

    struct TuningSample
    {
        float time{ 0.0f };
        float commanded{ 0.0f };
        float measured{ 0.0f };
        float torque{ 0.0f };
    };

    struct StepMetrics
    {
        std::optional<float> riseTime;
        float overshootPercent{ 0.0f };
        std::optional<float> settlingTime;
        float steadyStateError{ 0.0f };
    };

    struct TrackingMetrics
    {
        float rms{ 0.0f };
        float peak{ 0.0f };
    };

    StepMetrics MeasureStep(const std::vector<TuningSample>& samples, float start, float amplitude);
    TrackingMetrics MeasureTracking(const std::vector<TuningSample>& samples);
    void WriteCsv(const std::vector<TuningSample>& samples, std::ostream& output);
}
