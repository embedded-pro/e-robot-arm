#include "tools/simulator/view_models/FrameStatistics.hpp"
#include <algorithm>
#include <cmath>
#include <format>

namespace simulator
{
    namespace
    {
        constexpr std::chrono::nanoseconds window{ std::chrono::seconds{ 1 } };
        constexpr double minimumFramesPerSecond{ 30.0 };
        constexpr double frameBudgetMilliseconds{ 1000.0 / 60.0 };

        double Milliseconds(std::chrono::nanoseconds duration)
        {
            return std::chrono::duration<double, std::milli>{ duration }.count();
        }
    }

    void FrameStatistics::Durations::Add(std::chrono::nanoseconds duration)
    {
        total += duration;
        longest = std::max(longest, duration);
        ++count;
    }

    double FrameStatistics::Durations::MeanMilliseconds() const
    {
        return count == 0 ? 0.0 : Milliseconds(total) / count;
    }

    FrameStatistics::FrameStatistics(std::chrono::nanoseconds start)
        : windowStart(start)
    {}

    void FrameStatistics::AddGuiFrame(std::chrono::nanoseconds duration)
    {
        gui.Add(duration);
    }

    void FrameStatistics::AddRender(std::chrono::nanoseconds duration)
    {
        render.Add(duration);
    }

    bool FrameStatistics::Advance(std::chrono::nanoseconds now)
    {
        const auto elapsed = now - windowStart;
        if (elapsed < window)
            return false;

        Summarise(elapsed);
        windowStart = now;
        gui = {};
        render = {};
        return true;
    }

    const std::string& FrameStatistics::Text() const
    {
        return text;
    }

    ui::theme::StatusLevel FrameStatistics::Level() const
    {
        return level;
    }

    void FrameStatistics::Summarise(std::chrono::nanoseconds elapsed)
    {
        const auto framesPerSecond = render.count / std::chrono::duration<double>{ elapsed }.count();
        const auto decimals = framesPerSecond < 10.0 ? 1 : 0;
        const auto view = render.count == 0 ? std::string{ "3-D idle" }
                                            : std::format("3-D {:.{}f} fps · render {:.1f} ms", framesPerSecond, decimals, render.MeanMilliseconds());
        const auto guiText = gui.count == 0 ? std::string{ "GUI —" }
                                            : std::format("GUI {:.1f} ms (max {:.1f} ms)", gui.MeanMilliseconds(), Milliseconds(gui.longest));
        text = view + " · " + guiText;

        const auto slowView = render.count != 0 && framesPerSecond < minimumFramesPerSecond;
        const auto slowGui = Milliseconds(gui.longest) > frameBudgetMilliseconds;
        level = slowView || slowGui ? ui::theme::StatusLevel::Warning : ui::theme::StatusLevel::Neutral;
    }
}
