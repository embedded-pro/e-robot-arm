#pragma once

#include "ui/theme/Theme.hpp"
#include <chrono>
#include <cstdint>
#include <string>

namespace simulator
{
    class FrameStatistics
    {
    public:
        explicit FrameStatistics(std::chrono::nanoseconds start);

        void AddGuiFrame(std::chrono::nanoseconds duration);
        void AddRender(std::chrono::nanoseconds duration);
        bool Advance(std::chrono::nanoseconds now);

        const std::string& Text() const;
        ui::theme::StatusLevel Level() const;

    private:
        struct Durations
        {
            std::chrono::nanoseconds total{};
            std::chrono::nanoseconds longest{};
            uint32_t count{ 0 };

            void Add(std::chrono::nanoseconds duration);
            double MeanMilliseconds() const;
        };

        void Summarise(std::chrono::nanoseconds elapsed);

        std::chrono::nanoseconds windowStart;
        Durations gui;
        Durations render;
        std::string text{ "3-D — · GUI —" };
        ui::theme::StatusLevel level{ ui::theme::StatusLevel::Neutral };
    };
}
