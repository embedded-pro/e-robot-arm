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
