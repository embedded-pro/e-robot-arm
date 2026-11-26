#include "tools/simulator/view_models/ControlRules.hpp"
#include <format>
#include <optional>

namespace simulator
{
    namespace
    {
        ControlState Enabled()
        {
            return { true, {} };
        }

        ControlState Disabled(std::string reason)
        {
            return { false, std::move(reason) };
        }

        template<class Predicate>
        std::optional<std::size_t> FirstJoint(const arm_host::ArmSnapshot& snapshot, Predicate matches)
        {
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                if (matches(snapshot.readings[joint]))
                    return joint;
            return std::nullopt;
        }

        bool Running(const arm::JointReading& reading)
        {
            return reading.state == can::FocMotorState::running;
        }

        bool Faulted(const arm::JointReading& reading)
        {
            return reading.state == can::FocMotorState::fault;
        }

        std::string JointText(std::size_t joint, std::string_view what)
        {
            return std::format("Joint {} {}", joint + 1, what);
        }

        std::optional<std::string> MotionBlocked(const arm_host::ArmSnapshot& snapshot)
        {
            if (!snapshot.targetReady)
                return "Not connected";
            if (snapshot.contracts == can::ContractCompatibility::unknown)
                return "Contracts not checked";
            if (snapshot.contracts == can::ContractCompatibility::incompatible)
                return "Contracts incompatible";
            if (snapshot.tripped)
                return "Tripped";
            if (snapshot.holding)
                return "Holding: re-arm first";
            return std::nullopt;
        }

        ControlState Check(std::optional<std::string> blocked)
        {
            return blocked ? Disabled(std::move(*blocked)) : Enabled();
        }

        ControlState Move(const arm_host::ArmSnapshot& snapshot, std::string_view targetProblem)
        {
            using enum arm::ArmState;
            if (!snapshot.targetReady)
                return Disabled("Not connected");
            if (snapshot.state == tripped)
                return Disabled("Tripped");
            if (snapshot.state == holding)
                return Disabled("Holding: re-arm first");
            if (snapshot.state != enabled && snapshot.state != moving)
                return Disabled("Enable the arm first");
            if (arm_host::FreeMode(snapshot))
                return Disabled("Free mode: select a tracking controller");
            if (!targetProblem.empty())
                return Disabled(std::string{ targetProblem });
            return Enabled();
        }

        ControlState BringUp(const arm_host::ArmSnapshot& snapshot)
        {
            using enum arm::ArmState;
            if (!snapshot.targetReady)
                return Disabled("Not connected");
            switch (snapshot.state)
            {
                case unpowered:
                case incompatible:
                case misconfigured:
                    return Enabled();
                case checking:
                    return Disabled("Checking contracts");
                case tripped:
                    return Disabled("Tripped: re-arm first");
                case holding:
                    return Disabled("Holding: re-arm first");
                default:
                    return Disabled("Already brought up");
            }
        }

        ControlState Enable(const arm_host::ArmSnapshot& snapshot)
        {
            using enum arm::ArmState;
            if (!snapshot.targetReady)
                return Disabled("Not connected");
            switch (snapshot.state)
            {
                case ready:
                    return Enabled();
                case unpowered:
                case incompatible:
                    return Disabled("Bring the arm up first");
                case checking:
                    return Disabled("Checking contracts");
                case waitingForCalibration:
                    return Disabled("Waiting for calibration");
                case configuring:
                    return Disabled("Configuring motor controllers");
                case misconfigured:
                    return Disabled("Misconfigured: bring the arm up again");
                case enabling:
                    return Disabled("Enabling");
                case disabling:
                    return Disabled("Disabling");
                case tripped:
                    return Disabled("Tripped");
                case holding:
                    return Disabled("Holding: re-arm first");
                default:
                    return Disabled("Already enabled");
            }
        }

        ControlState Disable(const arm_host::ArmSnapshot& snapshot)
        {
            if (snapshot.state == arm::ArmState::enabled || snapshot.state == arm::ArmState::holding)
                return Enabled();
            if (snapshot.state == arm::ArmState::moving)
                return Disabled("Moving: abort first");
            return Disabled("Not enabled");
        }

        ControlState Commissioning(const arm_host::ArmSnapshot& snapshot)
        {
            if (snapshot.target.kind != arm_host::TargetKind::real)
                return Disabled("Not the real arm");
            if (DrivesJoints(snapshot))
                return Disabled("Disable the arm first");
            return Enabled();
        }

        ControlState Rearm(const arm_host::ArmSnapshot& snapshot)
        {
            if (snapshot.state != arm::ArmState::tripped && snapshot.state != arm::ArmState::holding)
                return Disabled("Not tripped or holding");
            if (snapshot.hardwareStopOpen)
                return Disabled("Hardware stop open: close it first");
            if (const auto joint = FirstJoint(snapshot, Faulted); joint.has_value())
                return Disabled(JointText(*joint, "in fault: clear it first"));
            return Enabled();
        }

        ControlState WhileConnected(const arm_host::ArmSnapshot& snapshot, bool allowedWhileMoving)
        {
            if (!snapshot.targetReady)
                return Disabled("Not connected");
            if (snapshot.moving && !allowedWhileMoving)
                return Disabled("Moving");
            return Enabled();
        }
    }

    bool DrivesJoints(const arm_host::ArmSnapshot& snapshot)
    {
        const auto state = snapshot.state;
        return snapshot.moving || state == arm::ArmState::enabling || state == arm::ArmState::enabled || state == arm::ArmState::moving ||
               state == arm::ArmState::disabling || state == arm::ArmState::holding;
    }

    ControlState Evaluate(Control control, const arm_host::ArmSnapshot& snapshot, std::string_view targetProblem)
    {
        switch (control)
        {
            case Control::emergencyStop:
                return snapshot.targetReady ? Enabled() : Disabled("No target");
            case Control::checkContracts:
                return WhileConnected(snapshot, false);
            case Control::torqueMode:
                return snapshot.moving ? Disabled("Moving") : Check(MotionBlocked(snapshot));
            case Control::start:
                return Check(MotionBlocked(snapshot));
            case Control::stop:
                return FirstJoint(snapshot, Running).has_value() ? Enabled() : Disabled("No joint running");
            case Control::clearFaults:
                return FirstJoint(snapshot, Faulted).has_value() ? Enabled() : Disabled("No joint in fault");
            case Control::rearm:
                return Rearm(snapshot);
            case Control::move:
                return Move(snapshot, targetProblem);
            case Control::abort:
                return snapshot.moving ? Enabled() : Disabled("No move running");
            case Control::target:
                return DrivesJoints(snapshot) ? Disabled("Disable the arm first") : Enabled();
            case Control::bringUp:
                return BringUp(snapshot);
            case Control::enable:
                return Enable(snapshot);
            case Control::disable:
                return Disable(snapshot);
            case Control::commissioning:
                return Commissioning(snapshot);
        }
        return Disabled("Unknown control");
    }
}
