#include "tools/simulator/view_models/RecentProfiles.hpp"
#include "gmock/gmock.h"
#include <set>
#include <string>

namespace
{
    class RecentProfilesTest
        : public testing::Test
    {
    public:
        std::set<std::string> existing{ "/arms/a.json", "/arms/b.json", "/arms/c.json", "/arms/d.json", "/arms/e.json", "/arms/f.json" };
        simulator::RecentProfiles recent{ [this](const std::string& path)
            {
                return existing.contains(path);
            } };
    };
}

TEST_F(RecentProfilesTest, lists_the_newest_first_by_file_name_with_the_path_as_tooltip)
{
    recent.Add("/arms/a.json");
    recent.Add("/arms/b.json");

    const auto entries = recent.Entries();
    ASSERT_EQ(2u, entries.size());
    EXPECT_EQ("b.json", entries[0].label);
    EXPECT_EQ("/arms/b.json", entries[0].path);
    EXPECT_EQ("/arms/b.json", entries[0].tooltip);
    EXPECT_TRUE(entries[0].available);
    EXPECT_EQ("a.json", entries[1].label);
}

TEST_F(RecentProfilesTest, a_path_appears_once_and_moves_to_the_top_when_used_again)
{
    recent.Add("/arms/a.json");
    recent.Add("/arms/b.json");
    recent.Add("/arms/a.json");

    const auto entries = recent.Entries();
    ASSERT_EQ(2u, entries.size());
    EXPECT_EQ("/arms/a.json", entries[0].path);
    EXPECT_EQ("/arms/b.json", entries[1].path);
}

TEST_F(RecentProfilesTest, keeps_the_five_newest)
{
    for (const auto* path : { "/arms/a.json", "/arms/b.json", "/arms/c.json", "/arms/d.json", "/arms/e.json", "/arms/f.json" })
        recent.Add(path);

    const auto entries = recent.Entries();
    ASSERT_EQ(5u, entries.size());
    EXPECT_EQ("/arms/f.json", entries.front().path);
    EXPECT_EQ("/arms/b.json", entries.back().path);
}

TEST_F(RecentProfilesTest, a_missing_file_stays_listed_but_unavailable_and_says_why)
{
    recent.Add("/arms/gone.json");

    const auto entries = recent.Entries();
    ASSERT_EQ(1u, entries.size());
    EXPECT_FALSE(entries[0].available);
    EXPECT_EQ("File not found: /arms/gone.json", entries[0].tooltip);
}

TEST_F(RecentProfilesTest, restores_a_stored_list_and_clears_it)
{
    recent.Restore({ "/arms/c.json", "/arms/c.json", "", "/arms/a.json" });

    EXPECT_EQ((std::vector<std::string>{ "/arms/c.json", "/arms/a.json" }), recent.Paths());

    recent.Clear();
    EXPECT_TRUE(recent.Entries().empty());
}

TEST_F(RecentProfilesTest, the_window_title_names_the_profile_between_the_product_and_the_target)
{
    EXPECT_EQ("e-robot-arm Simulator — bench arm — Simulated joints", simulator::WindowTitle("bench arm", "Simulated joints"));
    EXPECT_EQ("e-robot-arm Simulator — Simulated joints", simulator::WindowTitle("", "Simulated joints"));
}
