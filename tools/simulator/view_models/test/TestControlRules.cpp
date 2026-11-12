#include "tools/simulator/view_models/ControlRules.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"

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
    tripped.readings[2].state = can::FocMotorState::fault;

    ExpectDisabled(Control::start, tripped, "Tripped");
    ExpectDisabled(Control::move, tripped, "Tripped");
    ExpectDisabled(Control::rearm, tripped, "Joint 3 in fault: clear it first");
    ExpectEnabled(Control::clearFaults, tripped);

    tripped.readings[2].state = can::FocMotorState::ready;
    ExpectEnabled(Control::rearm, tripped);
    ExpectDisabled(Control::rearm, simulator::test::Running(), "Not tripped");
}

TEST_F(ControlRulesTest, move_needs_every_joint_running_and_valid_targets)
{
    auto oneStopped = simulator::test::Running();
    oneStopped.readings[4].state = can::FocMotorState::ready;

    ExpectDisabled(Control::move, oneStopped, "Joint 5 not running");
    ExpectDisabled(Control::move, simulator::test::Running(), "J2 outside its limits", "J2 outside its limits");
    ExpectEnabled(Control::move, simulator::test::Running());
}

TEST_F(ControlRulesTest, a_running_move_allows_abort_and_blocks_what_would_disturb_it)
{
    auto moving = simulator::test::Running();
    moving.moving = true;

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
