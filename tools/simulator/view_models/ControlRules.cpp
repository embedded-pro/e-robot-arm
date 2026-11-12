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

        bool NotRunning(const arm::JointReading& reading)
        {
            return !Running(reading);
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
            return std::nullopt;
        }

        ControlState Check(std::optional<std::string> blocked)
        {
            return blocked ? Disabled(std::move(*blocked)) : Enabled();
        }

        ControlState Move(const arm_host::ArmSnapshot& snapshot, std::string_view targetProblem)
        {
            if (auto blocked = MotionBlocked(snapshot))
                return Disabled(std::move(*blocked));
            if (snapshot.moving)
                return Disabled("Moving");
            if (const auto joint = FirstJoint(snapshot, NotRunning); joint.has_value())
                return Disabled(JointText(*joint, "not running"));
            if (!targetProblem.empty())
                return Disabled(std::string{ targetProblem });
            return Enabled();
        }

        ControlState Rearm(const arm_host::ArmSnapshot& snapshot)
        {
            if (!snapshot.tripped)
                return Disabled("Not tripped");
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

    ControlState Evaluate(Control control, const arm_host::ArmSnapshot& snapshot, std::string_view targetProblem)
    {
        switch (control)
        {
            case Control::emergencyStop:
                return snapshot.targetReady ? Enabled() : Disabled("No target");
            case Control::checkContracts:
                return WhileConnected(snapshot, false);
            case Control::positionMode:
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
                return snapshot.moving ? Disabled("Moving") : Enabled();
        }
        return Disabled("Unknown control");
    }
}
