#include "tools/arm_console/ConsoleOptions.hpp"
#include "gmock/gmock.h"
#include <string>
#include <vector>

namespace
{
    class ConsoleOptionsTest
        : public testing::Test
    {
    public:
        std::optional<arm_console::ConsoleOptions> Parse(std::vector<std::string> arguments) const
        {
            arguments.insert(arguments.begin(), "e-robot-arm-console");
            return arm_console::ParseOptions(arguments);
        }
    };
}

TEST_F(ConsoleOptionsTest, defaults_to_the_virtual_arm_bus_at_one_megabit_with_nodes_one_to_six)
{
    const auto options = Parse({});

    ASSERT_TRUE(options.has_value());
    EXPECT_EQ("virtual:arm", options->bus);
    EXPECT_EQ(1000000u, options->bitrate);
    EXPECT_EQ((arm::JointNodeIds{ 1, 2, 3, 4, 5, 6 }), options->nodes);
}

TEST_F(ConsoleOptionsTest, reads_the_bus_the_bitrate_and_the_nodes)
{
    const auto options = Parse({ "--bus", "can0", "--bitrate", "500000", "--nodes", "11,12,13,14,15,16" });

    ASSERT_TRUE(options.has_value());
    EXPECT_EQ("can0", options->bus);
    EXPECT_EQ(500000u, options->bitrate);
    EXPECT_EQ((arm::JointNodeIds{ 11, 12, 13, 14, 15, 16 }), options->nodes);
}

TEST_F(ConsoleOptionsTest, refuses_unknown_options_missing_values_and_a_node_list_that_is_not_six_distinct_ids)
{
    EXPECT_FALSE(Parse({ "--speed", "3" }).has_value());
    EXPECT_FALSE(Parse({ "--bus" }).has_value());
    EXPECT_FALSE(Parse({ "--bitrate", "fast" }).has_value());
    EXPECT_FALSE(Parse({ "--nodes", "1,2,3" }).has_value());
    EXPECT_FALSE(Parse({ "--nodes", "1,2,3,4,5,5" }).has_value());
    EXPECT_FALSE(Parse({ "--nodes", "0,2,3,4,5,6" }).has_value());
    EXPECT_FALSE(Parse({ "--nodes", "1,2,3,4,5,4096" }).has_value());
}

TEST_F(ConsoleOptionsTest, the_usage_names_every_option)
{
    const auto usage = arm_console::OptionsUsage();

    EXPECT_THAT(usage, testing::HasSubstr("--bus"));
    EXPECT_THAT(usage, testing::HasSubstr("--bitrate"));
    EXPECT_THAT(usage, testing::HasSubstr("--nodes"));
}
