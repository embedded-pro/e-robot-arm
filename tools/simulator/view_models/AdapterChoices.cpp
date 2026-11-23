#include "tools/simulator/view_models/AdapterChoices.hpp"
#include <algorithm>
#include <format>

namespace simulator
{
    namespace
    {
        AdapterChoices WithLastUsed(AdapterChoices choices, std::string_view lastUsed)
        {
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

    AdapterChoices SearchingForAdapters(std::string_view lastUsed)
    {
        return WithLastUsed({ {}, 0, "Searching for CAN adapters on this computer…", AdapterSearch::searching }, lastUsed);
    }

    AdapterChoices ChooseAdapter(std::span<const std::string> detected, std::string_view lastUsed)
    {
        AdapterChoices choices{ { detected.begin(), detected.end() }, 0, "CAN adapter (detected on this computer, or type another):", AdapterSearch::found };
        if (detected.empty())
        {
            choices.label = "No CAN adapter detected: type one, for example can0 or slcan:/dev/ttyACM0";
            choices.search = AdapterSearch::noneFound;
        }
        return WithLastUsed(std::move(choices), lastUsed);
    }

    AdapterChoices AdapterSearchTimedOut(std::string_view lastUsed)
    {
        return WithLastUsed({ {},
                                0,
                                std::format("The adapter search did not finish in {} s: type an adapter, or Search Again", adapterSearchSeconds),
                                AdapterSearch::timedOut },
            lastUsed);
    }
}
