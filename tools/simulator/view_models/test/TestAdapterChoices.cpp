#include "tools/simulator/view_models/AdapterChoices.hpp"
#include "gmock/gmock.h"

TEST(AdapterChoicesTest, lists_the_detected_adapters_and_selects_the_last_one_used)
{
    const std::vector<std::string> detected{ "can0", "can1", "slcan:/dev/ttyACM0" };

    const auto choices = simulator::ChooseAdapter(detected, "can1");

    EXPECT_EQ(detected, choices.items);
    EXPECT_EQ(1, choices.current);
    EXPECT_EQ("CAN adapter (detected on this computer, or type another):", choices.label);
}

TEST(AdapterChoicesTest, keeps_the_last_adapter_used_when_it_is_not_detected_now)
{
    const std::vector<std::string> detected{ "can0" };

    const auto choices = simulator::ChooseAdapter(detected, "slcan:/dev/ttyUSB0");

    EXPECT_EQ((std::vector<std::string>{ "can0", "slcan:/dev/ttyUSB0" }), choices.items);
    EXPECT_EQ(1, choices.current);
}

TEST(AdapterChoicesTest, says_so_when_nothing_is_detected)
{
    const auto choices = simulator::ChooseAdapter({}, "");

    EXPECT_TRUE(choices.items.empty());
    EXPECT_EQ(0, choices.current);
    EXPECT_EQ("No CAN adapter detected: type one, for example can0 or slcan:/dev/ttyACM0", choices.label);
}

TEST(AdapterChoicesTest, keeps_found_none_found_searching_and_timed_out_apart)
{
    const std::vector<std::string> none;
    const std::vector<std::string> one{ "can0" };

    EXPECT_EQ(simulator::AdapterSearch::found, simulator::ChooseAdapter(one, "").search);
    EXPECT_EQ(simulator::AdapterSearch::noneFound, simulator::ChooseAdapter(none, "").search);
    EXPECT_EQ(simulator::AdapterSearch::searching, simulator::SearchingForAdapters("").search);
    EXPECT_EQ(simulator::AdapterSearch::timedOut, simulator::AdapterSearchTimedOut("").search);
}

TEST(AdapterChoicesTest, while_searching_the_last_adapter_used_is_offered_at_once)
{
    const auto choices = simulator::SearchingForAdapters("slcan:/dev/ttyACM0");

    EXPECT_EQ((std::vector<std::string>{ "slcan:/dev/ttyACM0" }), choices.items);
    EXPECT_EQ(0, choices.current);
    EXPECT_EQ("Searching for CAN adapters on this computer…", choices.label);
    EXPECT_TRUE(simulator::SearchingForAdapters("").items.empty());
}

TEST(AdapterChoicesTest, a_search_that_did_not_finish_says_so_rather_than_that_nothing_was_found)
{
    const auto choices = simulator::AdapterSearchTimedOut("can0");

    EXPECT_EQ((std::vector<std::string>{ "can0" }), choices.items);
    EXPECT_EQ("The adapter search did not finish in 3 s: type an adapter, or Search Again", choices.label);
    EXPECT_NE(simulator::ChooseAdapter(std::vector<std::string>{}, "can0").label, choices.label);
}
