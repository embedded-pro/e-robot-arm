#include "tools/arm_host/DefaultArmConfig.hpp"
#include "tools/arm_host/ProfileFile.hpp"
#include "gmock/gmock.h"
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace
{
    class ProfileFileTest
        : public testing::Test
    {
    public:
        ~ProfileFileTest() override
        {
            std::filesystem::remove(path);
            std::filesystem::remove_all(Saving());
        }

        std::filesystem::path Saving() const
        {
            return std::filesystem::path{ path }.concat(".saving");
        }

        std::string Contents() const
        {
            std::ifstream file{ path, std::ios::binary };
            return { std::istreambuf_iterator<char>{ file }, std::istreambuf_iterator<char>{} };
        }

        std::filesystem::path path{ std::filesystem::temp_directory_path() / "e-robot-arm-profile-file-test.json" };
    };
}

TEST_F(ProfileFileTest, a_written_profile_reads_back)
{
    ASSERT_EQ("", arm_host::WriteProfileFile(path, arm_host::DefaultArmProfile({ 1, 2, 3, 4, 5, 6 })));

    const auto read = arm_host::ReadProfileFile(path);
    ASSERT_TRUE(std::holds_alternative<arm::ArmProfile>(read));
    EXPECT_EQ(6, std::get<arm::ArmProfile>(read).config.nodes[5]);
}

TEST_F(ProfileFileTest, reports_a_file_it_cannot_write_read_or_accept)
{
    EXPECT_EQ("cannot write /no/such/directory/profile.json", arm_host::WriteProfileFile("/no/such/directory/profile.json", arm_host::DefaultArmProfile({ 1, 2, 3, 4, 5, 6 })));
    EXPECT_EQ("cannot open /no/such/profile.json", std::get<std::string>(arm_host::ReadProfileFile("/no/such/profile.json")));

    std::ofstream{ path } << std::string(70 * 1024, ' ');
    EXPECT_EQ(path.string() + " is larger than 64 KiB", std::get<std::string>(arm_host::ReadProfileFile(path)));

    std::ofstream{ path } << "{ \"format\": \"something else\" }";
    EXPECT_EQ("format: must be \"e-robot-arm profile\"", std::get<std::string>(arm_host::ReadProfileFile(path)));
}

TEST_F(ProfileFileTest, refuses_a_file_just_over_the_limit_and_accepts_one_at_it)
{
    std::ofstream{ path, std::ios::binary } << std::string(64 * 1024 + 1, ' ');
    EXPECT_EQ(path.string() + " is larger than 64 KiB", std::get<std::string>(arm_host::ReadProfileFile(path)));

    std::ofstream{ path, std::ios::binary } << std::string(64 * 1024, ' ');
    EXPECT_NE(path.string() + " is larger than 64 KiB", std::get<std::string>(arm_host::ReadProfileFile(path)));
}

TEST_F(ProfileFileTest, a_save_that_fails_keeps_the_previous_profile_byte_for_byte)
{
    auto previous = arm_host::DefaultArmProfile({ 1, 2, 3, 4, 5, 6 });
    ASSERT_EQ("", arm_host::WriteProfileFile(path, previous));
    const auto before = Contents();
    std::filesystem::create_directory(Saving());

    const auto problem = arm_host::WriteProfileFile(path, arm_host::DefaultArmProfile({ 11, 12, 13, 14, 15, 16 }));

    EXPECT_EQ("cannot write " + path.string(), problem);
    EXPECT_EQ(before, Contents());
    EXPECT_TRUE(std::filesystem::is_directory(Saving()));
}

TEST_F(ProfileFileTest, a_save_that_cannot_replace_the_destination_keeps_it_and_removes_the_temporary_file)
{
    std::filesystem::create_directory(path);
    std::ofstream{ path / "keep" } << "kept";

    const auto problem = arm_host::WriteProfileFile(path, arm_host::DefaultArmProfile({ 1, 2, 3, 4, 5, 6 }));

    EXPECT_EQ(0u, problem.rfind("cannot replace " + path.string() + ": ", 0)) << problem;
    EXPECT_TRUE(std::filesystem::exists(path / "keep"));
    EXPECT_FALSE(std::filesystem::exists(Saving()));
    std::filesystem::remove_all(path);
}

TEST_F(ProfileFileTest, a_successful_save_replaces_the_profile_and_leaves_no_temporary_file)
{
    ASSERT_EQ("", arm_host::WriteProfileFile(path, arm_host::DefaultArmProfile({ 1, 2, 3, 4, 5, 6 })));
    ASSERT_EQ("", arm_host::WriteProfileFile(path, arm_host::DefaultArmProfile({ 11, 12, 13, 14, 15, 16 })));

    EXPECT_FALSE(std::filesystem::exists(Saving()));
    const auto read = arm_host::ReadProfileFile(path);
    ASSERT_TRUE(std::holds_alternative<arm::ArmProfile>(read));
    EXPECT_EQ(16, std::get<arm::ArmProfile>(read).config.nodes[5]);
}
