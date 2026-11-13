#include "tools/simulator/view_models/ToolReadout.hpp"
#include "tools/simulator/view_models/JointRows.hpp"
#include <array>
#include <format>
#include <string_view>

namespace simulator
{
    namespace
    {
        constexpr std::string_view placeholder{ "—" };

        float Shown(float value)
        {
            return value > -5.0e-4f && value < 5.0e-4f ? 0.0f : value;
        }

        std::string Names(const arm::Singularities& singularities)
        {
            const std::array<std::pair<bool, std::string_view>, 3> all{ { { singularities.wrist, "wrist" }, { singularities.elbow, "elbow" },
                { singularities.shoulder, "shoulder" } } };
            std::string names;
            for (const auto& [near, name] : all)
                if (near)
                    names += (names.empty() ? "" : ", ") + std::string{ name };
            return names;
        }
    }

    ToolReadout ToolReadoutOf(const arm_host::ArmSnapshot& snapshot)
    {
        if (!ModelIsLive(snapshot))
            return { std::string{ placeholder }, std::string{ placeholder }, std::string{ placeholder }, std::string{ placeholder }, ui::theme::StatusLevel::Neutral, {} };

        const auto& pose = snapshot.toolPose;
        ToolReadout readout{ std::format("x {:.3f}  y {:.3f}  z {:.3f} m", Shown(pose.x), Shown(pose.y), Shown(pose.z)),
            std::format("roll {:.3f}  pitch {:.3f}  yaw {:.3f} rad", Shown(pose.roll), Shown(pose.pitch), Shown(pose.yaw)),
            std::format("{:.2e}", snapshot.manipulability), std::string{ placeholder }, ui::theme::StatusLevel::Neutral, {} };

        if (snapshot.singularities.Any())
        {
            const auto names = Names(snapshot.singularities);
            const bool several = names.find(',') != std::string::npos;
            readout.singularity = names;
            readout.singularityLevel = ui::theme::StatusLevel::Warning;
            readout.warning = "Near " + names + (several ? " singularities" : " singularity");
        }
        return readout;
    }
}
