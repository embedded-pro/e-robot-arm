#include "tools/arm_host/ProfileFile.hpp"
#include "infra/stream/StdStringOutputStream.hpp"
#include <fstream>
#include <system_error>

namespace arm_host
{
    namespace
    {
        constexpr std::size_t maximumFileSize{ 64 * 1024 };

        std::filesystem::path SavingPath(const std::filesystem::path& path)
        {
            return std::filesystem::path{ path }.concat(".saving");
        }

        bool WriteWhole(const std::filesystem::path& path, const std::string& text)
        {
            std::ofstream file{ path, std::ios::binary | std::ios::trunc };
            if (!file)
                return false;
            file.write(text.data(), static_cast<std::streamsize>(text.size()));
            file.close();
            return !file.fail();
        }
    }

    std::variant<arm::ArmProfile, std::string> ReadProfileFile(const std::filesystem::path& path)
    {
        std::ifstream file{ path, std::ios::binary };
        if (!file)
            return "cannot open " + path.string();

        std::string text(maximumFileSize + 1, '\0');
        file.read(text.data(), static_cast<std::streamsize>(text.size()));
        text.resize(static_cast<std::size_t>(file.gcount()));
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

        const auto saving = SavingPath(path);
        std::error_code ignored;
        if (!WriteWhole(saving, stream.Storage()))
        {
            if (!std::filesystem::is_directory(saving, ignored))
                std::filesystem::remove(saving, ignored);
            return "cannot write " + path.string();
        }

        std::error_code renamed;
        std::filesystem::rename(saving, path, renamed);
        if (renamed)
        {
            std::filesystem::remove(saving, ignored);
            return "cannot replace " + path.string() + ": " + renamed.message();
        }
        return {};
    }
}
