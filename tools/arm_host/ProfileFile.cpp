#include "tools/arm_host/ProfileFile.hpp"
#include "infra/stream/StdStringOutputStream.hpp"
#include <fstream>
#include <iterator>

namespace arm_host
{
    namespace
    {
        constexpr std::size_t maximumFileSize{ 64 * 1024 };
    }

    std::variant<arm::ArmProfile, std::string> ReadProfileFile(const std::filesystem::path& path)
    {
        std::ifstream file{ path, std::ios::binary };
        if (!file)
            return "cannot open " + path.string();

        std::string text{ std::istreambuf_iterator<char>{ file }, std::istreambuf_iterator<char>{} };
        if (text.size() > maximumFileSize)
            return path.string() + " is larger than 64 KiB";

        infra::BoundedString::WithStorage<arm::profileProblemSize> problem;
        auto profile = arm::ReadArmProfile(infra::BoundedConstString{ text.data(), text.size() }, problem);
        if (!profile)
            return std::string{ problem.begin(), problem.end() };
        return *profile;
    }

    std::string WriteProfileFile(const std::filesystem::path& path, const arm::ArmProfile& profile)
    {
        infra::StdStringOutputStream::WithStorage stream;
        arm::WriteArmProfile(profile, stream);

        std::ofstream file{ path, std::ios::binary | std::ios::trunc };
        file << stream.Storage();
        file.close();
        if (!file)
            return "cannot write " + path.string();
        return {};
    }
}
