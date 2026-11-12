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

    ArmConsole::ArmConsole(arm::SafetySupervisor& safety, arm::JointMotion& motion, std::ostream& output)
        : arm::JointBusObserver(safety)
        , arm::JointMotionObserver(motion)
        , safety(safety)
        , motion(motion)
        , output(output)
    {}

    bool ArmConsole::Execute(const std::string& line)
    {
        std::istringstream arguments{ line };
        std::string command;
        if (!(arguments >> command))
            return true;

        if (command == "quit")
            return false;

        if (!Dispatch(command, arguments))
            Usage();
        return true;
    }

    bool ArmConsole::Dispatch(const std::string& command, std::istringstream& arguments)
    {
        if (command == "contracts")
            Answer(safety.QueryContracts());
        else if (command == "start")
            Answer(safety.Start());
        else if (command == "stop")
            Answer(safety.Stop());
        else if (command == "estop")
            Answer(safety.EmergencyStop());
        else if (command == "clear")
            Answer(safety.ClearFault());
        else if (command == "rearm")
            Answer(safety.Rearm());
        else if (command == "abort")
        {
            motion.Abort();
            Answer(true);
        }
        else if (command == "status")
            Status();
        else if (command == "mode")
            return Mode(arguments);
        else if (command == "move")
            return Move(arguments);
        else
            return false;

        return true;
    }

    bool ArmConsole::Mode(std::istringstream& arguments)
    {
        std::string name;
        arguments >> name;
        if (name == "torque")
            Answer(safety.SelectMode(can::FocMotorMode::torque));
        else if (name == "speed")
            Answer(safety.SelectMode(can::FocMotorMode::speed));
        else if (name == "position")
            Answer(safety.SelectMode(can::FocMotorMode::position));
        else
            return false;

        return true;
    }

    bool ArmConsole::Move(std::istringstream& arguments)
    {
        arm::JointPositions target{};
        for (auto& position : target)
        {
            float value{};
            if (!(arguments >> value))
                return false;
            position = foc::Radians{ value };
        }

        std::string rest;
        if (arguments >> rest)
            return false;

        Answer(motion.MoveTo(target));
        return true;
    }

    void ArmConsole::Status()
    {
        output << std::fixed << std::setprecision(3);
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto& reading = safety.Reading(joint);
            output << "joint " << joint + 1 << ": position " << reading.position.Value() << " rad, speed " << reading.speed.Value()
                   << " rad/s, state " << Name(stateNames, reading.state) << ", fault " << Name(faultNames, reading.fault) << "\n";
        }
        output << "safety: " << (safety.Tripped() ? "tripped" : "armed") << "\n";
        output << "motion: " << (motion.Moving() ? "moving" : "idle") << "\n";
    }

    void ArmConsole::Answer(bool accepted)
    {
        output << (accepted ? "ok" : "refused") << "\n";
    }

    void ArmConsole::Usage()
    {
        output << "usage: contracts | mode torque|speed|position | start | stop | estop | clear | rearm | move q1 q2 q3 q4 q5 q6 | abort | status | quit\n";
    }

    void ArmConsole::OnContractsResolved(can::ContractCompatibility compatibility)
    {
        output << "contracts " << (compatibility == can::ContractCompatibility::compatible ? "compatible" : "incompatible") << "\n";
    }

    void ArmConsole::OnJointFault(std::size_t joint, can::FocFaultCode fault)
    {
        output << "joint " << joint + 1 << " faulted: " << Name(faultNames, fault) << "\n";
    }

    void ArmConsole::OnJointSilent(std::size_t joint)
    {
        output << "joint " << joint + 1 << " silent\n";
    }

    void ArmConsole::OnMoveDone()
    {
        output << "move done\n";
    }

    void ArmConsole::OnMoveAborted()
    {
        output << "move aborted\n";
    }
}
