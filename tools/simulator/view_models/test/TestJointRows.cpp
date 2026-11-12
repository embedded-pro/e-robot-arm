#include "tools/simulator/view_models/JointRows.hpp"
#include "tools/simulator/view_models/test/SnapshotBuilder.hpp"
#include "gmock/gmock.h"

namespace
{
    class JointRowsTest
        : public testing::Test
    {};
}

TEST_F(JointRowsTest, without_a_target_shows_placeholders_and_neutral_status)
{
    const auto rows = simulator::JointRows({}, simulator::test::Limits());

    EXPECT_EQ("1", rows[0].number);
    EXPECT_EQ("6", rows[5].number);
    EXPECT_EQ("—", rows[0].position);
    EXPECT_EQ("—", rows[0].state);
    EXPECT_EQ(ui::theme::StatusLevel::Neutral, rows[0].level);
}

TEST_F(JointRowsTest, formats_readings_with_units_and_fixed_precision)
{
    auto snapshot = simulator::test::Running();
    snapshot.readings[1].position = foc::Radians{ 1.0f };
    snapshot.readings[1].speed = foc::RadiansPerSecond{ -0.126f };

    const auto rows = simulator::JointRows(snapshot, simulator::test::Limits());

    EXPECT_EQ("1.000 rad", rows[1].position);
    EXPECT_EQ("-0.13 rad/s", rows[1].speed);
    EXPECT_EQ("Running", rows[1].state);
    EXPECT_EQ("—", rows[1].fault);
    EXPECT_EQ(ui::theme::StatusLevel::Ok, rows[1].level);
    EXPECT_NEAR(0.75f, rows[1].limitFraction, 1.0e-5f);
}

TEST_F(JointRowsTest, a_faulted_joint_names_its_fault_at_fault_level)
{
    auto snapshot = simulator::test::Running();
    snapshot.readings[3].state = can::FocMotorState::fault;
    snapshot.readings[3].fault = can::FocFaultCode::overCurrent;

    const auto rows = simulator::JointRows(snapshot, simulator::test::Limits());

    EXPECT_EQ("Fault", rows[3].state);
    EXPECT_EQ("Overcurrent", rows[3].fault);
    EXPECT_EQ(ui::theme::StatusLevel::Fault, rows[3].level);
}

TEST_F(JointRowsTest, the_limit_bar_stays_within_its_ends)
{
    auto snapshot = simulator::test::Running();
    snapshot.readings[0].position = foc::Radians{ 5.0f };
    snapshot.readings[1].position = foc::Radians{ -5.0f };

    const auto rows = simulator::JointRows(snapshot, simulator::test::Limits());

    EXPECT_NEAR(1.0f, rows[0].limitFraction, 1.0e-5f);
    EXPECT_NEAR(0.0f, rows[1].limitFraction, 1.0e-5f);
}

TEST_F(JointRowsTest, a_silent_joint_shows_no_values_and_says_it_is_silent)
{
    auto snapshot = simulator::test::Running();
    snapshot.readings[4].position = foc::Radians{ 0.7f };
    snapshot.readings[4].silent = true;

    const auto rows = simulator::JointRows(snapshot, simulator::test::Limits());

    EXPECT_EQ("—", rows[4].position);
    EXPECT_EQ("—", rows[4].speed);
    EXPECT_EQ("Silent", rows[4].state);
    EXPECT_EQ(ui::theme::StatusLevel::Fault, rows[4].level);
}
