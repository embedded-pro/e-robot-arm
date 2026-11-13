#include "tools/simulator/view_models/ControlRules.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"
#include <array>
#include <string>
#include <utility>

namespace
{
    using simulator::Control;

    class ControlRulesTest
        : public testing::Test
    {
    public:
        void ExpectDisabled(Control control, const arm_host::ArmSnapshot& snapshot, const std::string& reason, std::string_view targetProblem = {})
        {
            const auto state = simulator::Evaluate(control, snapshot, targetProblem);
            EXPECT_FALSE(state.enabled);
            EXPECT_EQ(reason, state.reason);
        }

        void ExpectEnabled(Control control, const arm_host::ArmSnapshot& snapshot)
        {
            const auto state = simulator::Evaluate(control, snapshot);
            EXPECT_TRUE(state.enabled) << state.reason;
            EXPECT_TRUE(state.reason.empty());
        }
    };
}

TEST_F(ControlRulesTest, without_a_target_only_the_target_selector_works)
{
    const arm_host::ArmSnapshot disconnected{};

    ExpectDisabled(Control::emergencyStop, disconnected, "No target");
    ExpectDisabled(Control::checkContracts, disconnected, "Not connected");
    ExpectDisabled(Control::positionMode, disconnected, "Not connected");
    ExpectDisabled(Control::move, disconnected, "Not connected");
    ExpectEnabled(Control::target, disconnected);
}

TEST_F(ControlRulesTest, the_emergency_stop_is_enabled_in_every_connected_state)
{
    auto tripped = simulator::test::Running();
    tripped.tripped = true;
    tripped.moving = true;

    ExpectEnabled(Control::emergencyStop, simulator::test::Connected());
    ExpectEnabled(Control::emergencyStop, tripped);
}

TEST_F(ControlRulesTest, motion_waits_for_compatible_contracts)
{
    auto incompatible = simulator::test::Connected();
    incompatible.contracts = can::ContractCompatibility::incompatible;

    ExpectDisabled(Control::positionMode, simulator::test::Connected(), "Contracts not checked");
    ExpectDisabled(Control::start, incompatible, "Contracts incompatible");
    ExpectEnabled(Control::positionMode, simulator::test::Compatible());
    ExpectEnabled(Control::start, simulator::test::Compatible());
}

TEST_F(ControlRulesTest, a_trip_blocks_motion_and_allows_rearm_once_no_joint_is_in_fault)
{
    auto tripped = simulator::test::Running();
    tripped.tripped = true;
    tripped.state = arm::ArmState::tripped;
    tripped.readings[2].state = can::FocMotorState::fault;

    ExpectDisabled(Control::start, tripped, "Tripped");
    ExpectDisabled(Control::move, tripped, "Tripped");
    ExpectDisabled(Control::rearm, tripped, "Joint 3 in fault: clear it first");
    ExpectEnabled(Control::clearFaults, tripped);

    tripped.readings[2].state = can::FocMotorState::ready;
    ExpectEnabled(Control::rearm, tripped);
    ExpectDisabled(Control::rearm, simulator::test::Running(), "Not tripped");
}

TEST_F(ControlRulesTest, move_needs_an_enabled_arm_and_valid_targets)
{
    ExpectDisabled(Control::move, simulator::test::Compatible(), "Enable the arm first");
    ExpectDisabled(Control::move, simulator::test::Running(), "J2 outside its limits", "J2 outside its limits");
    ExpectEnabled(Control::move, simulator::test::Running());
}

TEST_F(ControlRulesTest, a_running_move_allows_abort_and_blocks_what_would_disturb_it)
{
    auto moving = simulator::test::Running();
    moving.moving = true;
    moving.state = arm::ArmState::moving;

    ExpectEnabled(Control::abort, moving);
    ExpectDisabled(Control::move, moving, "Moving");
    ExpectDisabled(Control::checkContracts, moving, "Moving");
    ExpectDisabled(Control::positionMode, moving, "Moving");
    ExpectDisabled(Control::target, moving, "Moving");
    ExpectDisabled(Control::abort, simulator::test::Running(), "No move running");
}

TEST_F(ControlRulesTest, stop_and_clear_faults_need_something_to_act_on)
{
    ExpectDisabled(Control::stop, simulator::test::Compatible(), "No joint running");
    ExpectEnabled(Control::stop, simulator::test::Running());
    ExpectDisabled(Control::clearFaults, simulator::test::Running(), "No joint in fault");
}

TEST_F(ControlRulesTest, bring_up_is_offered_while_the_arm_is_unpowered_or_its_contracts_are_incompatible)
{
    ExpectDisabled(Control::bringUp, arm_host::ArmSnapshot{}, "Not connected");
    ExpectEnabled(Control::bringUp, simulator::test::Connected());

    auto incompatible = simulator::test::Connected();
    incompatible.state = arm::ArmState::incompatible;
    ExpectEnabled(Control::bringUp, incompatible);

    auto checking = simulator::test::Connected();
    checking.state = arm::ArmState::checking;
    ExpectDisabled(Control::bringUp, checking, "Checking contracts");
    ExpectDisabled(Control::bringUp, simulator::test::Compatible(), "Already brought up");
}

TEST_F(ControlRulesTest, enable_is_offered_only_when_the_arm_is_ready_and_says_why_otherwise)
{
    ExpectEnabled(Control::enable, simulator::test::Compatible());
    ExpectDisabled(Control::enable, simulator::test::Connected(), "Bring the arm up first");

    auto waiting = simulator::test::Compatible();
    waiting.state = arm::ArmState::waitingForCalibration;
    ExpectDisabled(Control::enable, waiting, "Waiting for calibration");
    ExpectDisabled(Control::enable, simulator::test::Running(), "Already enabled");

    auto tripped = simulator::test::Compatible();
    tripped.state = arm::ArmState::tripped;
    ExpectDisabled(Control::enable, tripped, "Tripped");

    const std::array<std::pair<arm::ArmState, std::string>, 3> busy{ {
        { arm::ArmState::checking, "Checking contracts" },
        { arm::ArmState::enabling, "Enabling" },
        { arm::ArmState::disabling, "Disabling" },
    } };
    for (const auto& [state, reason] : busy)
    {
        auto snapshot = simulator::test::Compatible();
        snapshot.state = state;
        ExpectDisabled(Control::enable, snapshot, reason);
    }
}

TEST_F(ControlRulesTest, disable_is_offered_while_enabled_and_asks_to_abort_a_move_first)
{
    ExpectEnabled(Control::disable, simulator::test::Running());
    ExpectDisabled(Control::disable, simulator::test::Compatible(), "Not enabled");

    auto moving = simulator::test::Running();
    moving.state = arm::ArmState::moving;
    ExpectDisabled(Control::disable, moving, "Moving: abort first");
}
