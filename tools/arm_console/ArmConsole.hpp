#ifndef TOOLS_ARM_CONSOLE_ARM_CONSOLE_HPP
#define TOOLS_ARM_CONSOLE_ARM_CONSOLE_HPP

#include "tools/arm_host/ArmCommands.hpp"
#include "tools/arm_host/ArmEvent.hpp"
#include <cstdint>
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
        quit
    };

    struct ExecuteResult
    {
        Outcome outcome;
        uint32_t command{ 0 };
    };

    class ArmConsole
    {
    public:
        ArmConsole(arm_host::ArmCommands& arm, std::ostream& output);

        ExecuteResult Execute(const std::string& line);
        void Print(const arm_host::ArmEvent& event);

    private:
        std::optional<arm_host::ArmCommand> Parse(std::string_view command, std::istringstream& arguments) const;
        ExecuteResult Profile(std::string_view command, std::istringstream& arguments);
        void ArmControllers();
        ExecuteResult ArmController(std::istringstream& arguments);
        void Kinematics(std::string_view command, std::istringstream& arguments);
        void PrintPose(const arm::Placement& pose, const arm::Singularities& singularities);
        std::optional<arm_host::ArmCommand> Mode(std::istringstream& arguments) const;
        std::optional<arm_host::ArmCommand> Move(std::istringstream& arguments) const;
        void Status();
        void Usage();

        arm_host::ArmCommands& arm;
        std::ostream& output;
    };
}

#endif
