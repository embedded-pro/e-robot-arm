#include "tools/arm_console/ArmConsole.hpp"
#include "numerical/math/Math.hpp"
#include "tools/arm_host/ProfileFile.hpp"
#include <algorithm>
#include <array>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>

namespace arm_console
{
    namespace
    {
        std::string_view SafetyText(const arm_host::ArmSnapshot& snapshot)
        {
            if (snapshot.tripped)
                return "tripped";
            if (snapshot.holding)
                return "holding";
            return "armed";
        }

        constexpr std::array<const char*, 7> stateNames{ "idle", "running", "fault", "calibrating", "partialCalibration", "ready", "stopping" };
        constexpr std::array<const char*, 14> faultNames{ "none", "overCurrent", "overVoltage", "underVoltage", "overTemperature", "sensorFault", "stall",
            "phaseMissing", "hardwareFault", "calibrationFailed", "overSpeed", "followingError", "commandTimeout", "positionLimit" };

        template<std::size_t Size, class Enum>
        const char* Name(const std::array<const char*, Size>& names, Enum value)
        {
            const auto index = static_cast<std::size_t>(value);
            return index < names.size() ? names[index] : "unknown";
        }

        std::string StopText(const std::optional<arm_host::StopReport>& stop)
        {
            if (!stop.has_value())
                return {};
            std::string cause{ stop->reason };
            if (cause.empty())
                cause = stop->fault == can::FocFaultCode::none ? "silent" : Name(faultNames, stop->fault);
            const auto* category = stop->category == arm_host::StopCategory::emergency ? ", emergency stop (category 0): " : ", protective stop (category 2): ";
            if (stop->joint == arm_host::noJoint)
                return category + cause;
            return std::format("{}joint {} {}", category, stop->joint + 1, cause);
        }

        constexpr std::array<std::string_view, 2> motionProfileNames{ "trapezoid", "s-curve" };
        constexpr std::array<std::string_view, 2> frameNames{ "base", "tool" };
        constexpr std::array<std::string_view, 6> axisNames{ "x", "y", "z", "rx", "ry", "rz" };

        std::optional<infra::Duration> HeldFor(std::istringstream& arguments)
        {
            float seconds{ 1.0f };
            std::string rest;
            if (!(arguments >> seconds))
            {
                if (!arguments.eof())
                    return std::nullopt;
                seconds = 1.0f;
            }
            else if (arguments >> rest)
                return std::nullopt;
            return std::chrono::duration_cast<infra::Duration>(std::chrono::duration<float>(seconds));
        }

        std::optional<arm_host::AnyJog> ToolJogOf(std::string_view frame, std::istringstream& arguments)
        {
            std::string axis;
            float speed{ 0.0f };
            if (!(arguments >> axis >> speed))
                return std::nullopt;
            const auto foundAxis = std::ranges::find(axisNames, axis);
            if (foundAxis == axisNames.end())
                return std::nullopt;
            const auto chosenFrame = frame == frameNames[0] ? arm::JogFrame::base : arm::JogFrame::tool;
            return arm::ToolJog{ chosenFrame, static_cast<arm::ToolAxis>(foundAxis - axisNames.begin()), speed };
        }

        std::string JointList(const arm::JointFlags& joints)
        {
            std::string list;
            for (std::size_t joint = 0; joint != joints.size(); ++joint)
                if (joints[joint])
                    list += std::format("{}{}", list.empty() ? "" : ", ", joint + 1);
            return list;
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

        if (command == "load" || command == "save")
            return Profile(command, arguments);

        if (command == "controllers")
        {
            ArmControllers();
            return { Outcome::done };
        }

        if (command == "controller")
            return ArmController(arguments);

        if (command == "motion")
            return MotionProfile(arguments);

        if (command == "jog")
            return Jog(arguments);

        if (command == "teach" || command == "goto")
            return TeachPoint(command, arguments);

        if (command == "points")
        {
            Points();
            return { Outcome::done };
        }

        if (command == "tune")
            return Tune(arguments);

        if (command == "pose" || command == "fk" || command == "ik" || command == "gravity")
        {
            Kinematics(command, arguments);
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
        if (command == "align")
            return arm_host::AlignJoints{};
        if (command == "rearm")
            return arm_host::Rearm{};
        if (command == "abort")
            return arm_host::AbortMove{};
        if (command == "bringup")
            return arm_host::BringUp{};
        if (command == "enable")
            return arm_host::Enable{};
        if (command == "disable")
            return arm_host::Disable{};
        if (command == "mode")
            return Mode(arguments);
        if (command == "move")
            return Move(arguments);
        if (command == "step")
        {
            auto move = Move(arguments);
            if (const auto* target = move.has_value() ? std::get_if<arm_host::MoveTo>(&*move) : nullptr)
                return arm_host::StepTo{ target->target };
            return std::nullopt;
        }
        return std::nullopt;
    }

    ExecuteResult ArmConsole::Profile(std::string_view command, std::istringstream& arguments)
    {
        std::filesystem::path path;
        std::string rest;
        if (!(arguments >> path) || (arguments >> rest))
        {
            Usage();
            return { Outcome::done };
        }

        if (command == "save")
        {
            const auto snapshot = arm.Snapshot();
            arm::ArmProfile profile{ {}, snapshot.config };
            profile.name.assign(snapshot.profileName.substr(0, arm::profileNameSize));
            const auto problem = arm_host::WriteProfileFile(path, profile);
            output << (problem.empty() ? "saved " + path.string() : "save failed: " + problem) << "\n";
            return { Outcome::done };
        }

        auto read = arm_host::ReadProfileFile(path);
        if (const auto* problem = std::get_if<std::string>(&read))
        {
            output << "profile refused: " << *problem << "\n";
            return { Outcome::done };
        }

        return { Outcome::posted, arm.LoadProfile(std::get<arm::ArmProfile>(read)) };
    }

    void ArmConsole::ArmControllers()
    {
        for (const auto& controller : arm.Snapshot().armControllers)
            output << (controller.active ? "* " : "  ") << controller.name << (controller.enabled ? "" : " (disabled)") << "\n";
    }

    ExecuteResult ArmConsole::ArmController(std::istringstream& arguments)
    {
        std::string action;
        std::string name;
        std::string rest;
        if (!(arguments >> action >> name) || (arguments >> rest) || (action != "enable" && action != "disable" && action != "select"))
        {
            Usage();
            return { Outcome::done };
        }

        const auto controllers = arm.Snapshot().armControllers;
        const auto found = std::ranges::find(controllers, name, &arm_host::ArmControllerStatus::name);
        if (found == controllers.end())
        {
            output << "no arm controller " << name << "\n";
            return { Outcome::done };
        }

        const auto id = static_cast<arm::ArmControllerId>(found - controllers.begin());
        if (action == "enable")
            return { Outcome::posted, arm.Post(arm_host::EnableArmController{ id }) };
        if (action == "disable")
            return { Outcome::posted, arm.Post(arm_host::DisableArmController{ id }) };
        return { Outcome::posted, arm.Post(arm_host::SelectArmController{ id }) };
    }

    ExecuteResult ArmConsole::MotionProfile(std::istringstream& arguments)
    {
        std::string name;
        std::string rest;
        if (!(arguments >> name))
        {
            output << "motion profile: " << motionProfileNames[static_cast<std::size_t>(arm.Snapshot().config.motionProfile)] << "\n";
            return { Outcome::done };
        }

        const auto found = std::ranges::find(motionProfileNames, name);
        if (found == motionProfileNames.end() || (arguments >> rest))
        {
            Usage();
            return { Outcome::done };
        }

        return { Outcome::posted, arm.Post(arm_host::SelectMotionProfile{ static_cast<arm::MotionProfile>(found - motionProfileNames.begin()) }) };
    }

    ExecuteResult ArmConsole::Jog(std::istringstream& arguments)
    {
        std::string first;
        std::optional<arm_host::AnyJog> jog;
        if (arguments >> first)
        {
            if (std::ranges::find(frameNames, first) != frameNames.end())
                jog = ToolJogOf(first, arguments);
            else if (first.size() == 1 && first[0] >= '1' && first[0] <= '6')
            {
                float speed{ 0.0f };
                if (arguments >> speed)
                    jog = arm::JointJog{ static_cast<std::size_t>(first[0] - '1'), foc::RadiansPerSecond{ speed } };
            }
        }

        const auto duration = jog.has_value() ? HeldFor(arguments) : std::nullopt;
        if (!duration.has_value())
        {
            Usage();
            return { Outcome::done };
        }

        return { Outcome::posted, arm.Post(arm_host::HoldJog{ *jog, *duration }) };
    }

    ExecuteResult ArmConsole::TeachPoint(std::string_view command, std::istringstream& arguments)
    {
        std::string name;
        std::string rest;
        if (!(arguments >> name) || (arguments >> rest))
        {
            Usage();
            return { Outcome::done };
        }

        if (command == "teach")
            return { Outcome::posted, arm.Post(arm_host::Teach{ name }) };
        return { Outcome::posted, arm.Post(arm_host::GoTo{ name }) };
    }

    void ArmConsole::Points()
    {
        const auto config = arm.Snapshot().config;
        if (config.teachPointCount == 0)
            output << "no teach points\n";
        for (std::size_t point = 0; point != config.teachPointCount; ++point)
        {
            const auto& taught = config.teachPoints[point];
            output << std::string_view{ taught.name.data(), taught.name.size() } << ":";
            for (const auto& joint : taught.joints)
                output << std::format(" {:.3f}", joint.Value());
            output << " rad\n";
        }
    }

    void ArmConsole::Kinematics(std::string_view command, std::istringstream& arguments)
    {
        const auto snapshot = arm.Snapshot();
        std::array<float, arm::jointCount> values{};
        const bool wantsValues = command == "fk" || command == "ik";
        for (auto& value : values)
            if (wantsValues && !(arguments >> value))
            {
                Usage();
                return;
            }

        std::string rest;
        if (arguments >> rest)
        {
            Usage();
            return;
        }

        if (command == "pose")
            return PrintPose(snapshot.toolPose, snapshot.singularities);
        if (command == "gravity")
        {
            output << "gravity torque:";
            for (const auto& torque : snapshot.gravityTorque)
                output << std::format(" {:.3f}", torque.Value() > -5.0e-4f && torque.Value() < 5.0e-4f ? 0.0f : torque.Value());
            output << " N·m\n";
            return;
        }

        const arm::ArmModel model{ snapshot.config.geometry, snapshot.config.mass, snapshot.config.positionLimits };
        arm::JointPositions current{};
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            current[joint] = command == "fk" ? foc::Radians{ values[joint] } : snapshot.readings[joint].position;

        if (command == "fk")
            return PrintPose(arm::ToPlacement(model.ToolPose(current)), model.Near(current));

        const auto result = model.Inverse(arm::ToTransform({ values[0], values[1], values[2], values[3], values[4], values[5] }), current);
        if (!result.joints.has_value())
        {
            output << "ik refused: " << (result.failure == arm::IkFailure::unreachable ? "unreachable" : "outside joint limits") << "\n";
            return;
        }

        output << "joints:";
        for (const auto& joint : *result.joints)
            output << std::format(" {:.3f}", joint.Value());
        output << " rad\n";
    }

    void ArmConsole::PrintPose(const arm::Placement& pose, const arm::Singularities& singularities)
    {
        const auto shown = [](float value)
        {
            return value > -5.0e-4f && value < 5.0e-4f ? 0.0f : value;
        };

        output << std::format("tool: x {:.3f} y {:.3f} z {:.3f} m, roll {:.3f} pitch {:.3f} yaw {:.3f} rad", shown(pose.x), shown(pose.y), shown(pose.z),
            shown(pose.roll), shown(pose.pitch), shown(pose.yaw));
        if (singularities.Any())
            output << ", near " << (singularities.wrist ? "wrist " : "") << (singularities.elbow ? "elbow " : "") << (singularities.shoulder ? "shoulder " : "")
                   << "singularity";
        output << "\n";
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
        output << "arm: " << arm_host::StateName(snapshot.state) << "\n";
        output << "safety: " << SafetyText(snapshot) << StopText(snapshot.stop) << "\n";
        if (const auto joints = JointList(snapshot.torqueClamped); !joints.empty())
            output << "torque limited: joints " << joints << "\n";
        output << "motion: " << (snapshot.moving ? "moving" : "idle") << "\n";
        output << "overruns: " << snapshot.overruns << "\n";
    }

    void ArmConsole::Usage()
    {
        output << "usage: bringup | enable | disable | move q1 q2 q3 q4 q5 q6 | step q1 q2 q3 q4 q5 q6 | tune step|move <joint> <rad> [csv] | rearm | contracts | mode torque|speed|position | start | stop | estop | clear | align | abort | load <file> | save <file> | controllers | controller enable|disable|select <name> | motion [trapezoid|s-curve] | jog <joint> <rad/s> [s] | jog base|tool x|y|z|rx|ry|rz <speed> [s] | teach <name> | goto <name> | points | pose | gravity | fk q1..q6 | ik x y z roll pitch yaw | status | quit\n";
    }

    ExecuteResult ArmConsole::Tune(std::istringstream& arguments)
    {
        std::string kind;
        std::size_t joint{ 0 };
        float amplitude{ 0.0f };
        std::filesystem::path csv;
        if (!(arguments >> kind >> joint >> amplitude) || (kind != "step" && kind != "move") || joint == 0)
        {
            Usage();
            return { Outcome::done };
        }
        arguments >> csv;

        std::scoped_lock lock{ tuningMutex };
        const tuning::TuningTest test{ .kind = kind == "step" ? tuning::TestKind::step : tuning::TestKind::outAndBack, .joint = joint - 1, .amplitude = amplitude };
        tuningRun.emplace(arm, test);
        if (const auto refusal = tuningRun->Start())
        {
            output << "tune refused: " << *refusal << "\n";
            tuningRun.reset();
            return { Outcome::done };
        }

        tuningCsv = csv;
        return { Outcome::posted, tuningRun->Command() };
    }

    void ArmConsole::Trace(const arm_host::TraceSample& sample)
    {
        std::scoped_lock lock{ tuningMutex };
        if (!tuningRun)
            return;
        tuningRun->OnTrace(sample);
        ReportTuning();
    }

    void ArmConsole::ReportTuning()
    {
        if (tuningRun->State() == tuning::RunState::failed)
            output << "tune failed: " << tuningRun->Failure() << "\n";
        if (tuningRun->State() != tuning::RunState::done)
        {
            if (tuningRun->State() == tuning::RunState::failed)
                tuningRun.reset();
            return;
        }

        const auto& test = tuningRun->Test();
        const auto tracking = tuningRun->Tracking();
        const auto seconds = [](const std::optional<float>& value)
        {
            return value.has_value() ? std::format("{:.3f} s", *value) : std::string{ "none" };
        };
        if (const auto step = tuningRun->Step())
            output << std::format("tune step joint {} by {:.3f} rad: rise {}, overshoot {:.1f} %, settling {}, steady-state error {:.4f} rad, ", test.joint + 1,
                test.amplitude, seconds(step->riseTime), step->overshootPercent, seconds(step->settlingTime), math::Abs(step->steadyStateError) < 5.0e-5f ? 0.0f : step->steadyStateError);
        else
            output << std::format("tune move joint {} by {:.3f} rad and back: ", test.joint + 1, test.amplitude);
        output << std::format("tracking RMS {:.4f} rad, peak {:.4f} rad\n", tracking.rms, tracking.peak);

        if (!tuningCsv.empty())
        {
            std::ofstream file{ tuningCsv };
            tuning::WriteCsv(tuningRun->Samples(), file);
            output << (file ? "tune samples written to " : "tune: cannot write ") << tuningCsv.string() << "\n";
        }
        tuningRun.reset();
    }

    void ArmConsole::Print(const arm_host::ArmEvent& event)
    {
        PrintEvent(event);
        std::scoped_lock lock{ tuningMutex };
        if (!tuningRun)
            return;
        tuningRun->OnEvent(event);
        ReportTuning();
    }

    void ArmConsole::PrintEvent(const arm_host::ArmEvent& event)
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
            case supervisionTrip:
                output << "joint " << event.joint + 1 << " " << event.reason << "\n";
                break;
            case protectiveStop:
                output << "protective stop, holding: joint " << event.joint + 1 << " " << event.reason << "\n";
                break;
            case hardwareStop:
                output << event.reason << "\n";
                break;
            case tripped:
                if (event.joint == arm_host::noJoint)
                    output << "tripped by the " << event.reason << "\n";
                else
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
            case stateChanged:
                output << "arm: " << arm_host::StateName(event.state);
                if (event.joint != arm_host::noJoint)
                    output << " (joint " << event.joint + 1 << " " << event.reason << ")";
                output << "\n";
                break;
            case controllerApplied:
                output << "joint " << event.joint + 1 << " controller applied\n";
                break;
            case controllerRead:
                output << "joint " << event.joint + 1 << " controller read\n";
                break;
            case controllerFailed:
                output << "joint " << event.joint + 1 << " controller failed: " << event.reason << "\n";
                break;
            case motorApplied:
                output << "joint " << event.joint + 1 << " motor applied\n";
                break;
            case motorRead:
                output << "joint " << event.joint + 1 << " motor read\n";
                break;
            case profileLoaded:
                output << "profile loaded: " << arm.Snapshot().profileName << "\n";
                break;
            case armControllerChanged:
                output << "arm controller: " << event.armController << "\n";
                break;
        }
    }
}
