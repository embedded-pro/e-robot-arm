#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace simulator
{
    struct RecentProfile
    {
        std::string path;
        std::string label;
        std::string tooltip;
        bool available{ false };
    };

    class RecentProfiles
    {
    public:
        static constexpr std::size_t capacity{ 5 };

        explicit RecentProfiles(std::function<bool(const std::string& path)> exists);

        void Add(const std::string& path);
        void Restore(const std::vector<std::string>& stored);
        void Clear();

        const std::vector<std::string>& Paths() const;
        std::vector<RecentProfile> Entries() const;

    private:
        std::function<bool(const std::string& path)> exists;
        std::vector<std::string> paths;
    };

    std::string WindowTitle(std::string_view profile, std::string_view target);
}
