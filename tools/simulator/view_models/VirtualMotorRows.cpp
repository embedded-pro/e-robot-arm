#include "tools/simulator/view_models/VirtualMotorRows.hpp"
#include <algorithm>
#include <format>

namespace simulator
{
    namespace
    {
        using ui::theme::StatusLevel;
        using virtual_arm::MotorPhase;

        bool ArmRuns(arm::ArmState state)
        {
            return state == arm::ArmState::enabling || state == arm::ArmState::enabled || state == arm::ArmState::moving || state == arm::ArmState::disabling;
        }

        std::pair<std::string, StatusLevel> Process(const virtual_arm::MotorReport& motor, std::size_t joint)
        {
            switch (motor.phase)
            {
                case MotorPhase::stopped:
                    return { "Stopped", StatusLevel::Neutral };
                case MotorPhase::starting:
                    return { "Starting…", StatusLevel::Neutral };
                case MotorPhase::up:
                    return { "Up", StatusLevel::Ok };
                case MotorPhase::ended:
                    return { std::format("Ended: exit code {} (see joint{}.log)", motor.exitCode.value_or(-1), joint + 1), StatusLevel::Fault };
            }
            return { "—", StatusLevel::Neutral };
        }

        std::string Hint(const arm_host::ArmSnapshot& snapshot)
        {
            if (!snapshot.targetReady)
                return "Restart: the virtual arm is starting";
            if (ArmRuns(snapshot.state))
                return "Restart: disable the arm first";
            return {};
        }
    }

    VirtualMotorsView ViewVirtualMotors(const arm_host::ArmSnapshot& snapshot)
    {
        VirtualMotorsView view;
        view.visible = snapshot.target.kind == arm_host::TargetKind::virtualArm;
        view.hint = Hint(snapshot);
        const bool mayRestart = view.hint.empty();

        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            auto& row = view.rows[joint];
            row.joint = std::format("J{}", joint + 1);
            row.motor = "—";
            row.process = "—";
            if (!snapshot.virtualMotors.has_value())
                continue;

            const auto& motor = snapshot.virtualMotors->motors[joint];
            if (!motor.plant.empty())
                row.motor = motor.plant;
            std::tie(row.process, row.level) = Process(motor, joint);
            row.restartEnabled = mayRestart && motor.phase != MotorPhase::starting;
        }

        if (snapshot.virtualMotors.has_value() && !snapshot.virtualMotors->folder.empty())
            view.folder = "Logs and memory: " + snapshot.virtualMotors->folder;
        return view;
    }

    std::string StartingVirtualMotors(const arm_host::ArmSnapshot& snapshot)
    {
        std::size_t up{ 0 };
        if (snapshot.virtualMotors.has_value())
            up = static_cast<std::size_t>(std::ranges::count(snapshot.virtualMotors->motors, MotorPhase::up, &virtual_arm::MotorReport::phase));
        return std::format("Starting virtual motors: {} of {} up", up, arm::jointCount);
    }
}
