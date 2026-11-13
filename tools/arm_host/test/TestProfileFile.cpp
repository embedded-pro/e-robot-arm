#include "tools/arm_host/DefaultArmConfig.hpp"
#include "tools/arm_host/ProfileFile.hpp"
#include "gmock/gmock.h"
#include <filesystem>
#include <fstream>
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
