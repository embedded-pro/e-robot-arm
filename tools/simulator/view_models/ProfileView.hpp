#ifndef TOOLS_SIMULATOR_VIEW_MODELS_PROFILE_VIEW_HPP
#define TOOLS_SIMULATOR_VIEW_MODELS_PROFILE_VIEW_HPP

#include "tools/arm_host/ArmSnapshot.hpp"
#include <array>
#include <string>

namespace simulator
{
    struct ProfileRow
    {
        std::string joint;
        std::string node;
        std::string ratio;
        std::string direction;
        std::string offset;
        std::string lower;
        std::string upper;
        std::string speed;
        std::string acceleration;
        std::string controller;

        bool operator==(const ProfileRow& other) const = default;
    };

    struct ProfileView
    {
        std::string name;
        std::string summary;
        std::array<ProfileRow, arm::jointCount> rows{};

        bool operator==(const ProfileView& other) const = default;
    };

    inline constexpr std::array<std::string_view, 10> profileColumns{ "Joint", "Node", "Ratio", "Dir", "Offset rad", "Lower rad", "Upper rad", "Speed rad/s",
        "Accel rad/s²", "Motor controller" };

    ProfileView DescribeProfile(const arm_host::ArmSnapshot& snapshot);
}

#endif
