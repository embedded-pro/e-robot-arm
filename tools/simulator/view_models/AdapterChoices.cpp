#include "tools/simulator/view_models/AdapterChoices.hpp"
#include <algorithm>

namespace simulator
{
    AdapterChoices ChooseAdapter(std::span<const std::string> detected, std::string_view lastUsed)
    {
        AdapterChoices choices{ { detected.begin(), detected.end() }, 0, "CAN adapter (detected on this computer, or type another):" };
        if (detected.empty())
            choices.label = "No CAN adapter detected: type one, for example can0 or slcan:/dev/ttyACM0";

        if (lastUsed.empty())
            return choices;

        auto last = std::ranges::find(choices.items, lastUsed);
        if (last == choices.items.end())
        {
            choices.items.emplace_back(lastUsed);
            last = std::prev(choices.items.end());
        }
        choices.current = static_cast<int>(std::distance(choices.items.begin(), last));
        return choices;
    }
}
