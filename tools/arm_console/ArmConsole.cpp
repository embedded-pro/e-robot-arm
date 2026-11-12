#include "tools/arm_console/ArmConsole.hpp"
#include <array>
#include <iomanip>

namespace arm_console
{
    namespace
    {
        constexpr std::array<const char*, 6> stateNames{ "idle", "running", "fault", "calibrating", "partialCalibration", "ready" };
        constexpr std::array<const char*, 14> faultNames{ "none", "overCurrent", "overVoltage", "underVoltage", "overTemperature", "sensorFault", "stall",
            "phaseMissing", "hardwareFault", "calibrationFailed", "overSpeed", "followingError", "commandTimeout", "positionLimit" };

        template<std::size_t Size, class Enum>
        const char* Name(const std::array<const char*, Size>& names, Enum value)
        {
            const auto index = static_cast<std::size_t>(value);
            return index < names.size() ? names[index] : "unknown";
        }
    }

    ArmConsole::ArmConsole(arm_host::ArmCommands& arm, std::ostream& output)
        : arm(arm)
        , output(output)
    {}

    ExecuteResult ArmConsole::Execute(const std::string& line)
    {
        std::istringstream arguments{ line };
        std::string command;
        if (!(arguments >> command))
            return { Outcome::done };

        if (command == "quit")
            return { Outcome::quit };

        if (command == "status")
        {
            Status();
            return { Outcome::done };
        }

        const auto parsed = Parse(command, arguments);
        if (!parsed)
        {
            Usage();
            return { Outcome::done };
        }

        return { Outcome::posted, arm.Post(*parsed) };
    }

    std::optional<arm_host::ArmCommand> ArmConsole::Parse(std::string_view command, std::istringstream& arguments) const
    {
        if (command == "contracts")
            return arm_host::CheckContracts{};
        if (command == "start")
            return arm_host::StartJoints{};
        if (command == "stop")
            return arm_host::StopJoints{};
        if (command == "estop")
            return arm_host::EmergencyStop{};
        if (command == "clear")
            return arm_host::ClearFaults{};
        if (command == "rearm")
            return arm_host::Rearm{};
        if (command == "abort")
            return arm_host::AbortMove{};
        if (command == "mode")
            return Mode(arguments);
        if (command == "move")
            return Move(arguments);
        return std::nullopt;
    }

    std::optional<arm_host::ArmCommand> ArmConsole::Mode(std::istringstream& arguments) const
    {
        std::string name;
        arguments >> name;
        if (name == "torque")
            return arm_host::SelectMode{ can::FocMotorMode::torque };
        if (name == "speed")
            return arm_host::SelectMode{ can::FocMotorMode::speed };
        if (name == "position")
            return arm_host::SelectMode{ can::FocMotorMode::position };
        return std::nullopt;
    }

    std::optional<arm_host::ArmCommand> ArmConsole::Move(std::istringstream& arguments) const
    {
        arm::JointPositions target{};
        for (auto& position : target)
        {
            float value{};
            if (!(arguments >> value))
                return std::nullopt;
            position = foc::Radians{ value };
        }

        std::string rest;
        if (arguments >> rest)
            return std::nullopt;

        return arm_host::MoveTo{ target };
    }

    void ArmConsole::Status()
    {
        const auto snapshot = arm.Snapshot();
        output << std::fixed << std::setprecision(3);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto& reading = snapshot.readings[joint];
            output << "joint " << joint + 1 << ": position " << reading.position.Value() << " rad, speed " << reading.speed.Value()
                   << " rad/s, state " << Name(stateNames, reading.state) << ", fault " << Name(faultNames, reading.fault) << (reading.silent ? ", silent" : "") << "\n";
        }
        output << "safety: " << (snapshot.tripped ? "tripped" : "armed") << "\n";
        output << "motion: " << (snapshot.moving ? "moving" : "idle") << "\n";
    }

    void ArmConsole::Usage()
    {
        output << "usage: contracts | mode torque|speed|position | start | stop | estop | clear | rearm | move q1 q2 q3 q4 q5 q6 | abort | status | quit\n";
    }

    void ArmConsole::Print(const arm_host::ArmEvent& event)
    {
        using enum arm_host::ArmEventKind;

        switch (event.kind)
        {
            case accepted:
                output << "ok\n";
                break;
            case refused:
                output << "refused: " << event.reason << "\n";
                break;
            case contractsResolved:
                output << "contracts " << (event.contracts == can::ContractCompatibility::compatible ? "compatible" : "incompatible") << "\n";
                break;
            case jointFault:
                output << "joint " << event.joint + 1 << " faulted: " << Name(faultNames, event.fault) << "\n";
                break;
            case jointSilent:
                output << "joint " << event.joint + 1 << " silent\n";
                break;
            case tripped:
                output << "tripped by joint " << event.joint + 1 << "\n";
                break;
            case moveDone:
                output << "move done\n";
                break;
            case moveAborted:
                output << "move aborted\n";
                break;
            case targetReady:
                output << "target ready\n";
                break;
            case targetFailed:
                output << "target failed: " << event.reason << "\n";
                break;
        }
    }
}
