#include "tools/tuning/TuningMetrics.hpp"
#include "gmock/gmock.h"
#include <cmath>
#include <sstream>

namespace
{
    constexpr float period{ 0.01f };
    constexpr float tolerance{ 1.0e-4f };

    class TuningMetricsTest
        : public testing::Test
    {
    public:
        void Response(std::size_t count, float start, float amplitude, float (*shape)(float time))
        {
            for (std::size_t index = 0; index != count; ++index)
            {
                const auto time = static_cast<float>(index) * period;
                samples.push_back({ time, start + amplitude, start + amplitude * shape(time), 0.5f });
            }
        }

        std::vector<tuning::TuningSample> samples;
    };

    float FirstOrder(float time)
    {
        return 1.0f - std::exp(-time / 0.1f);
    }

    float Ramp(float time)
    {
        return std::min(1.0f, time / 0.2f) * 1.2f - std::max(0.0f, std::min(0.2f, time - 0.2f));
    }

    float Sluggish(float time)
    {
        return 0.5f * std::min(1.0f, time);
    }
}

TEST_F(TuningMetricsTest, a_first_order_step_rises_in_2_2_time_constants_and_settles_in_about_four_without_overshoot)
{
    Response(200, 0.3f, 0.1f, FirstOrder);

    const auto metrics = tuning::MeasureStep(samples, 0.3f, 0.1f);

    ASSERT_TRUE(metrics.riseTime.has_value());
    EXPECT_NEAR(0.22f, *metrics.riseTime, 0.011f);
    EXPECT_NEAR(0.0f, metrics.overshootPercent, tolerance);
    ASSERT_TRUE(metrics.settlingTime.has_value());
    EXPECT_NEAR(0.39f, *metrics.settlingTime, 0.011f);
    EXPECT_NEAR(0.0f, metrics.steadyStateError, 1.0e-3f);
}

TEST_F(TuningMetricsTest, an_overshooting_step_reports_its_peak_in_percent_of_the_step_also_for_a_negative_step)
{
    Response(200, 0.0f, -0.2f, Ramp);

    const auto metrics = tuning::MeasureStep(samples, 0.0f, -0.2f);

    EXPECT_NEAR(20.0f, metrics.overshootPercent, 0.1f);
    EXPECT_NEAR(0.0f, metrics.steadyStateError, 1.0e-3f);
}

TEST_F(TuningMetricsTest, a_step_that_never_reaches_90_percent_has_no_rise_time_nor_settling_time_and_keeps_its_error)
{
    Response(200, 0.0f, 0.1f, Sluggish);

    const auto metrics = tuning::MeasureStep(samples, 0.0f, 0.1f);

    EXPECT_FALSE(metrics.riseTime.has_value());
    EXPECT_FALSE(metrics.settlingTime.has_value());
    EXPECT_NEAR(0.05f, metrics.steadyStateError, 1.0e-3f);
}

TEST_F(TuningMetricsTest, the_tracking_error_is_the_rms_and_the_peak_of_commanded_minus_measured)
{
    samples = { { 0.0f, 1.0f, 1.0f, 0.0f }, { 0.01f, 1.0f, 0.9f, 0.0f }, { 0.02f, 1.0f, 1.1f, 0.0f }, { 0.03f, 1.0f, 1.0f, 0.0f } };

    const auto metrics = tuning::MeasureTracking(samples);

    EXPECT_NEAR(std::sqrt(0.02f / 4.0f), metrics.rms, tolerance);
    EXPECT_NEAR(0.1f, metrics.peak, tolerance);
}

TEST_F(TuningMetricsTest, writes_a_header_and_one_row_per_sample_as_csv)
{
    samples = { { 0.0f, 0.5f, 0.25f, 1.5f }, { 0.01f, 0.5f, 0.5f, -2.0f, 7 } };
    std::ostringstream csv;

    tuning::WriteCsv(samples, csv);

    EXPECT_EQ("time_s,commanded_rad,measured_rad,tracking_error_rad,torque_nm,dropped_before\n"
              "0,0.5,0.25,0.25,1.5,0\n"
              "0.01,0.5,0.5,0,-2,7\n",
        csv.str());
}
