#pragma once

#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace simulator
{
    struct AdapterChoices
    {
        std::vector<std::string> items;
        int current{ 0 };
        std::string label;
    };

    AdapterChoices ChooseAdapter(std::span<const std::string> detected, std::string_view lastUsed);
}
