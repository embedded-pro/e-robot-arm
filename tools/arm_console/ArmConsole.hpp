#pragma once

#include "tools/arm_host/ArmCommands.hpp"
#include "tools/arm_host/ArmEvent.hpp"
#include "tools/arm_host/ArmSnapshot.hpp"
#include "tools/tuning/TuningRun.hpp"
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>

namespace arm_console
{
    enum class Outcome : uint8_t
    {
        done,
        posted,
        wait,
        failed,
        quit
    };

    struct ExecuteResult
    {
        Outcome outcome;
        uint32_t command{ 0 };
        std::chrono::milliseconds waitFor{};
        std::optional<arm::ArmState> waitState{};
    };

    class ArmConsole
    {
    public:
        ArmConsole(arm_host::ArmCommands& arm, std::ostream& output);

        ExecuteResult Execute(const std::string& line);
        void Print(const arm_host::ArmEvent& event);
        void Trace(const arm_host::TraceSample& sample);

    private:
        std::optional<arm_host::ArmCommand> Parse(std::string_view command, std::istringstream& arguments) const;
        ExecuteResult Profile(std::string_view command, std::istringstream& arguments);
        void ArmControllers();
        ExecuteResult ArmController(std::istringstream& arguments);
        ExecuteResult MotionProfile(std::istringstream& arguments);
        ExecuteResult Impedance(std::istringstream& arguments);
        ExecuteResult Adaptation(std::istringstream& arguments);
        ExecuteResult Jog(std::istringstream& arguments);
        ExecuteResult TeachPoint(std::string_view command, std::istringstream& arguments);
        ExecuteResult MoveJoints(std::istringstream& arguments);
        ExecuteResult MoveLine(std::istringstream& arguments);
        void Points();
        ExecuteResult Kinematics(std::string_view command, std::istringstream& arguments);
        void PrintPose(const arm::Placement& pose, const arm::Singularities& singularities);
        std::optional<arm_host::ArmCommand> Mode(std::istringstream& arguments) const;
        std::optional<arm_host::ArmCommand> Move(std::istringstream& arguments) const;
        void Status();
        ExecuteResult Wait(std::istringstream& arguments);
        ExecuteResult Tune(std::istringstream& arguments);
        void ReportTuning();
        void PrintEvent(const arm_host::ArmEvent& event);
        void Usage();

        arm_host::ArmCommands& arm;
        std::ostream& output;
        std::mutex tuningMutex;
        std::optional<tuning::TuningRun> tuningRun;
        std::filesystem::path tuningCsv;
    };
}
