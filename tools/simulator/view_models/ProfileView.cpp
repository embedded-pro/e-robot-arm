#include "tools/simulator/view_models/ProfileView.hpp"
#include <chrono>
#include <format>

namespace simulator
{
    namespace
    {
        constexpr std::array<std::string_view, 3> modeNames{ "torque", "speed", "position" };
        constexpr std::array<std::string_view, 4> currentNames{ "PID", "decoupled PID", "deadbeat", "sliding mode" };
        constexpr std::array<std::string_view, 4> speedNames{ "PID", "LQI", "ADRC", "two-DOF" };
        constexpr std::array<std::string_view, 5> positionNames{ "PID", "cascade P", "LQR", "LQI", "two-DOF" };

        std::string Controller(const std::optional<arm::ControllerSettings>& controller)
        {
            if (!controller.has_value())
                return "node's own";
            return std::format("{} / {} / {}", currentNames[static_cast<std::size_t>(controller->current)], speedNames[static_cast<std::size_t>(controller->speed)],
                positionNames[static_cast<std::size_t>(controller->position)]);
        }

        ProfileRow Row(const arm::ArmConfig& config, std::size_t joint)
        {
            const auto& transmission = config.transmissions[joint];
            const auto& limit = config.positionLimits[joint];
            const auto& motion = config.motionLimits[joint];
            return { std::format("J{}", joint + 1), std::format("{}", config.nodes[joint]), std::format("{:.2f}", transmission.ratio),
                transmission.direction == arm::JointDirection::positive ? "+" : "−", std::format("{:.3f}", transmission.offset.Value()),
                std::format("{:.3f}", limit.lower.Value()), std::format("{:.3f}", limit.upper.Value()), std::format("{:.3f}", motion.maxSpeed.Value()),
                std::format("{:.3f}", motion.maxAcceleration.Value()), Controller(config.controllers[joint]) };
        }
    }

    ProfileView DescribeProfile(const arm_host::ArmSnapshot& snapshot)
    {
        if (!snapshot.targetReady)
            return { "—", "No target", {} };

        const auto& config = snapshot.config;
        ProfileView view{ snapshot.profileName,
            std::format("Control period {} ms · Enable mode {} · Silent after {} · Command timeout {}",
                std::chrono::duration_cast<std::chrono::milliseconds>(config.controlPeriod).count(), modeNames[static_cast<std::size_t>(config.enableMode)],
                config.missedRequestsBeforeSilent, config.commandTimeoutPeriods),
            {} };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            view.rows[joint] = Row(config, joint);
        return view;
    }
}
