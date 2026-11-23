#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace simulator
{
    enum class AdapterSearch : uint8_t
    {
        searching,
        found,
        noneFound,
        timedOut
    };

    struct AdapterChoices
    {
        std::vector<std::string> items;
        int current{ 0 };
        std::string label;
        AdapterSearch search{ AdapterSearch::found };
    };

    inline constexpr int adapterSearchSeconds{ 3 };

    AdapterChoices SearchingForAdapters(std::string_view lastUsed);
    AdapterChoices ChooseAdapter(std::span<const std::string> detected, std::string_view lastUsed);
    AdapterChoices AdapterSearchTimedOut(std::string_view lastUsed);
}
