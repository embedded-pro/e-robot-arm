#include "tools/simulator/view_models/RecentProfiles.hpp"
#include <algorithm>
#include <format>

namespace simulator
{
    namespace
    {
        std::string FileName(std::string_view path)
        {
            const auto separator = path.find_last_of("/\\");
            return std::string{ separator == std::string_view::npos ? path : path.substr(separator + 1) };
        }
    }

    RecentProfiles::RecentProfiles(std::function<bool(const std::string& path)> exists)
        : exists(std::move(exists))
    {}

    void RecentProfiles::Add(const std::string& path)
    {
        if (path.empty())
            return;
        std::erase(paths, path);
        paths.insert(paths.begin(), path);
        if (paths.size() > capacity)
            paths.resize(capacity);
    }

    void RecentProfiles::Restore(const std::vector<std::string>& stored)
    {
        paths.clear();
        for (const auto& path : stored)
            if (!path.empty() && std::ranges::find(paths, path) == paths.end() && paths.size() != capacity)
                paths.push_back(path);
    }

    void RecentProfiles::Clear()
    {
        paths.clear();
    }

    const std::vector<std::string>& RecentProfiles::Paths() const
    {
        return paths;
    }

    std::vector<RecentProfile> RecentProfiles::Entries() const
    {
        std::vector<RecentProfile> entries;
        for (const auto& path : paths)
        {
            const bool available = exists && exists(path);
            entries.emplace_back(path, FileName(path), available ? path : "File not found: " + path, available);
        }
        return entries;
    }

    std::string WindowTitle(std::string_view profile, std::string_view target)
    {
        if (profile.empty())
            return std::format("e-robot-arm Simulator — {}", target);
        return std::format("e-robot-arm Simulator — {} — {}", profile, target);
    }
}
