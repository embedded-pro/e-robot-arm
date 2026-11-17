#include "tools/arm_host/test_doubles/ArmCommandsMock.hpp"
#include "tools/tuning/TuningRun.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    using namespace testing;
    using namespace std::chrono_literals;

    class TuningRunTest
        : public Test
    {
    public:
        TuningRunTest()
        {
            snapshot.state = arm::ArmState::enabled;
            arm::JointPositions commanded{};
            commanded[2] = foc::Radians{ 0.4f };
            snapshot.commanded = commanded;
            EXPECT_CALL(arm, Snapshot()).WillRepeatedly(Invoke([this]()
                {
                    return snapshot;
                }));
        }

        void Answer(arm_host::ArmEventKind kind, uint32_t id, std::string reason = {})
        {
            run.OnEvent(arm_host::ArmEvent{ .kind = kind, .command = id, .reason = std::move(reason) });
        }

        void Event(arm_host::ArmEventKind kind)
        {
            run.OnEvent(arm_host::ArmEvent{ .kind = kind });
        }

        void Trace(std::size_t count, float measured)
        {
            for (std::size_t index = 0; index != count; ++index)
            {
                arm_host::TraceSample sample{ .time = time };
                sample.commanded[2] = foc::Radians{ 0.5f };
                sample.measured[2] = foc::Radians{ measured };
                arm::JointTorques torques{};
                torques[2] = foc::NewtonMeter{ 1.5f };
                sample.torques = torques;
                run.OnTrace(sample);
                time += 10ms;
            }
        }

        StrictMock<arm_host::ArmCommandsMock> arm;
        arm_host::ArmSnapshot snapshot;
        infra::TimePoint time{};
        tuning::TuningRun run{ arm, tuning::TuningTest{ .kind = tuning::TestKind::step, .joint = 2, .amplitude = 0.1f, .record = 500ms } };
    };

    class OutAndBackTest
        : public TuningRunTest
    {
    public:
        tuning::TuningRun outAndBack{ arm, tuning::TuningTest{ .kind = tuning::TestKind::outAndBack, .joint = 2, .amplitude = -0.5f } };
    };
}

TEST_F(TuningRunTest, a_step_test_steps_one_joints_reference_from_where_it_is_commanded)
{
    arm::JointPositions target;
    EXPECT_CALL(arm, Post(_)).WillOnce(Invoke([&target](const arm_host::ArmCommand& command)
        {
            target = std::get<arm_host::StepTo>(command).target;
            return 7u;
        }));

    EXPECT_EQ(std::nullopt, run.Start());

    EXPECT_NEAR(0.5f, target[2].Value(), 1.0e-6f);
    EXPECT_NEAR(0.0f, target[1].Value(), 1.0e-6f);
    EXPECT_EQ(tuning::RunState::starting, run.State());
}

TEST_F(TuningRunTest, records_the_tested_joint_from_the_acceptance_for_its_time_then_is_done_with_step_metrics)
{
    EXPECT_CALL(arm, Post(_)).WillOnce(Return(7u));
    run.Start();
    Trace(3, 0.4f);
    EXPECT_TRUE(run.Samples().empty());

    Answer(arm_host::ArmEventKind::accepted, 7);
    Event(arm_host::ArmEventKind::moveDone);
    Trace(50, 0.5f);
    EXPECT_EQ(tuning::RunState::recording, run.State());
    Trace(1, 0.5f);

    EXPECT_EQ(tuning::RunState::done, run.State());
    ASSERT_EQ(51u, run.Samples().size());
    EXPECT_NEAR(0.0f, run.Samples().front().time, 1.0e-6f);
    EXPECT_NEAR(0.5f, run.Samples().back().time, 1.0e-6f);
    EXPECT_NEAR(1.5f, run.Samples().back().torque, 1.0e-6f);
    ASSERT_TRUE(run.Step().has_value());
    EXPECT_NEAR(0.0f, run.Step()->steadyStateError, 1.0e-4f);
    Trace(5, 0.5f);
    EXPECT_EQ(51u, run.Samples().size());
}

TEST_F(TuningRunTest, refuses_to_start_unless_the_arm_is_enabled_and_idle_with_a_step_of_at_most_0_2_rad)
{
    snapshot.state = arm::ArmState::ready;
    EXPECT_EQ("the arm is not enabled", run.Start());
    snapshot.state = arm::ArmState::enabled;
    snapshot.moving = true;
    EXPECT_EQ("the arm is moving", run.Start());

    tuning::TuningRun large{ arm, tuning::TuningTest{ .kind = tuning::TestKind::step, .joint = 2, .amplitude = 0.25f } };
    snapshot.moving = false;
    EXPECT_EQ("a step is not 0 and at most 0.2 rad", large.Start());
    tuning::TuningRun seventh{ arm, tuning::TuningTest{ .kind = tuning::TestKind::step, .joint = 6, .amplitude = 0.1f } };
    EXPECT_EQ("the joint is 1 to 6", seventh.Start());
    EXPECT_EQ(tuning::RunState::idle, large.State());
}

TEST_F(TuningRunTest, a_refused_command_an_aborted_move_or_a_trip_fails_the_run_with_the_reason)
{
    EXPECT_CALL(arm, Post(_)).WillOnce(Return(7u));
    run.Start();
    Answer(arm_host::ArmEventKind::refused, 7, "the arm is not enabled");
    EXPECT_EQ(tuning::RunState::failed, run.State());
    EXPECT_EQ("step refused: the arm is not enabled", run.Failure());

    tuning::TuningRun tripped{ arm, tuning::TuningTest{ .kind = tuning::TestKind::step, .joint = 2, .amplitude = 0.1f } };
    EXPECT_CALL(arm, Post(_)).WillOnce(Return(8u));
    tripped.Start();
    tripped.OnEvent(arm_host::ArmEvent{ .kind = arm_host::ArmEventKind::accepted, .command = 8 });
    tripped.OnEvent(arm_host::ArmEvent{ .kind = arm_host::ArmEventKind::tripped, .reason = "collision 25.0 N·m" });
    EXPECT_EQ(tuning::RunState::failed, tripped.State());
    EXPECT_EQ("tripped: collision 25.0 N·m", tripped.Failure());
}

TEST_F(OutAndBackTest, moves_out_then_back_and_records_until_half_a_second_after_the_return_with_tracking_metrics_only)
{
    EXPECT_CALL(arm, Post(_)).WillOnce(Invoke([](const arm_host::ArmCommand& command)
        {
            EXPECT_NEAR(-0.1f, std::get<arm_host::MoveTo>(command).target[2].Value(), 1.0e-6f);
            return 3u;
        }));
    EXPECT_EQ(std::nullopt, outAndBack.Start());
    outAndBack.OnEvent(arm_host::ArmEvent{ .kind = arm_host::ArmEventKind::accepted, .command = 3 });

    EXPECT_CALL(arm, Post(_)).WillOnce(Invoke([](const arm_host::ArmCommand& command)
        {
            EXPECT_NEAR(0.4f, std::get<arm_host::MoveTo>(command).target[2].Value(), 1.0e-6f);
            return 4u;
        }));
    outAndBack.OnEvent(arm_host::ArmEvent{ .kind = arm_host::ArmEventKind::moveDone });
    EXPECT_EQ(tuning::RunState::returning, outAndBack.State());
    outAndBack.OnEvent(arm_host::ArmEvent{ .kind = arm_host::ArmEventKind::accepted, .command = 4 });
    outAndBack.OnEvent(arm_host::ArmEvent{ .kind = arm_host::ArmEventKind::moveDone });
    EXPECT_EQ(tuning::RunState::settling, outAndBack.State());

    for (int index = 0; index != 51; ++index)
    {
        arm_host::TraceSample sample{ .time = time };
        sample.commanded[2] = foc::Radians{ 0.4f };
        sample.measured[2] = foc::Radians{ 0.38f };
        outAndBack.OnTrace(sample);
        time += 10ms;
    }

    EXPECT_EQ(tuning::RunState::done, outAndBack.State());
    EXPECT_FALSE(outAndBack.Step().has_value());
    EXPECT_NEAR(0.02f, outAndBack.Tracking().rms, 1.0e-4f);
    EXPECT_NEAR(0.0f, outAndBack.Samples().front().torque, 1.0e-6f);
}
