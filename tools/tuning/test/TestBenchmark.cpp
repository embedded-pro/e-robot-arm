#include "tools/arm_host/test_doubles/ArmCommandsMock.hpp"
#include "tools/tuning/Benchmark.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <sstream>
#include <vector>

namespace
{
    using namespace testing;
    using namespace std::chrono_literals;
    using arm::ArmControllerKind;
    using tuning::BenchmarkRunState;
    using tuning::BenchmarkState;

    class BenchmarkTest
        : public Test
    {
    public:
        BenchmarkTest()
        {
            snapshot.targetReady = true;
            snapshot.state = arm::ArmState::enabled;
            snapshot.armControllers = { { "gravity-pd", true, true }, { "computed-torque", true, false }, { "gravity-pid", true, false }, { "free", true, false, true } };
            for (auto& limit : snapshot.config.positionLimits)
                limit = { foc::Radians{ -2.0f }, foc::Radians{ 2.0f } };
            snapshot.config.gravityPid = { 15.0f, 0.8f };
            EXPECT_CALL(arm, Snapshot()).WillRepeatedly(Invoke([this]()
                {
                    return snapshot;
                }));
            EXPECT_CALL(arm, Post(_)).WillRepeatedly(Invoke([this](const arm_host::ArmCommand& command)
                {
                    posted.push_back(command);
                    return static_cast<uint32_t>(posted.size());
                }));
        }

        void Accept()
        {
            benchmark.OnEvent({ .kind = arm_host::ArmEventKind::accepted, .command = static_cast<uint32_t>(posted.size()) });
        }

        void Trace(float seconds, float error, float torque = 2.0f)
        {
            for (float elapsed = 0.0f; elapsed < seconds; elapsed += 0.01f)
            {
                arm_host::TraceSample sample{ .time = time };
                sample.commanded[1] = foc::Radians{ position + error };
                sample.measured[1] = foc::Radians{ position };
                arm::JointTorques torques{};
                torques[1] = foc::NewtonMeter{ torque };
                sample.torques = torques;
                benchmark.OnTrace(sample);
                time += 10ms;
                position += 0.001f;
            }
        }

        void RunPath()
        {
            for (std::size_t leg = 0; leg != 5; ++leg)
            {
                Accept();
                Trace(0.2f, 0.02f);
                benchmark.OnEvent({ .kind = arm_host::ArmEventKind::moveDone });
            }
        }

        template<class Command>
        std::size_t Count() const
        {
            return static_cast<std::size_t>(std::ranges::count_if(posted, [](const arm_host::ArmCommand& command)
                {
                    return std::holds_alternative<Command>(command);
                }));
        }

        StrictMock<arm_host::ArmCommandsMock> arm;
        arm_host::ArmSnapshot snapshot;
        std::vector<arm_host::ArmCommand> posted;
        infra::TimePoint time{};
        float position{ 0.0f };
        tuning::Benchmark benchmark{ arm, { ArmControllerKind::computedTorque, ArmControllerKind::gravityPid } };
    };
}

TEST(BenchmarkPathTest, runs_through_the_teach_points_or_four_fixed_poses_within_the_limits_and_back)
{
    arm::ArmConfig config{};
    for (auto& limit : config.positionLimits)
        limit = { foc::Radians{ -0.3f }, foc::Radians{ 0.3f } };
    arm::JointPositions start{};
    start[0] = foc::Radians{ 0.1f };

    const auto fixed = tuning::BenchmarkPath(config, start);
    ASSERT_EQ(5u, fixed.size());
    EXPECT_FLOAT_EQ(0.2f, fixed[0][0].Value());
    EXPECT_FLOAT_EQ(0.25f, fixed[0][4].Value());
    EXPECT_FLOAT_EQ(-0.25f, fixed[2][0].Value());
    EXPECT_EQ(start, fixed.back());

    config.teachPointCount = 2;
    config.teachPoints[1].joints[3] = foc::Radians{ 0.2f };
    const auto taught = tuning::BenchmarkPath(config, start);
    ASSERT_EQ(3u, taught.size());
    EXPECT_FLOAT_EQ(0.2f, taught[1][3].Value());
}

TEST_F(BenchmarkTest, refuses_to_start_without_controllers_an_enabled_arm_or_an_offered_tracking_controller)
{
    EXPECT_EQ("choose at least one controller", tuning::BenchmarkRefusal({}, snapshot));
    EXPECT_EQ("at most eight controllers", tuning::BenchmarkRefusal(std::vector<ArmControllerKind>(9, ArmControllerKind::gravityPd), snapshot));
    EXPECT_EQ("free tracks no reference", tuning::BenchmarkRefusal({ ArmControllerKind::free }, snapshot));
    EXPECT_EQ("slotine-li is not offered", tuning::BenchmarkRefusal({ ArmControllerKind::slotineLi }, snapshot));
    auto moving = snapshot;
    moving.moving = true;
    EXPECT_EQ("the arm is moving", tuning::BenchmarkRefusal({ ArmControllerKind::gravityPd }, moving));
    auto ready = snapshot;
    ready.state = arm::ArmState::ready;
    EXPECT_EQ("the arm is not enabled", tuning::BenchmarkRefusal({ ArmControllerKind::gravityPd }, ready));
    auto disconnected = snapshot;
    disconnected.targetReady = false;
    EXPECT_EQ("not connected", tuning::BenchmarkRefusal({ ArmControllerKind::gravityPd }, disconnected));
    EXPECT_EQ(std::nullopt, tuning::BenchmarkRefusal({ ArmControllerKind::gravityPd }, snapshot));
}

TEST_F(BenchmarkTest, runs_each_controller_on_the_path_after_a_hold_measures_it_and_restores_the_first_controller)
{
    ASSERT_EQ(std::nullopt, benchmark.Start());
    ASSERT_EQ(1u, posted.size());
    EXPECT_EQ(1u, std::get<arm_host::SelectArmController>(posted[0]).id);

    Accept();
    Trace(0.5f, 0.0f);
    EXPECT_EQ(0u, Count<arm_host::MoveTo>());
    Trace(0.6f, 0.0f);
    EXPECT_EQ(1u, Count<arm_host::MoveTo>());

    RunPath();
    EXPECT_EQ(5u, Count<arm_host::MoveTo>());
    Trace(0.5f, 0.001f);
    EXPECT_EQ(BenchmarkRunState::running, benchmark.Results()[0].state);
    Trace(0.6f, 0.001f);

    const auto& first = benchmark.Results()[0];
    EXPECT_EQ(BenchmarkRunState::done, first.state);
    EXPECT_NEAR(0.02f, first.metrics.peakError, 1.0e-5f);
    EXPECT_NEAR(2.0f, first.metrics.peakTorque, 1.0e-5f);
    EXPECT_GT(first.metrics.rmsError, 0.005f);
    EXPECT_LT(first.metrics.rmsError, 0.02f);
    EXPECT_GT(first.metrics.energy, 0.0f);
    ASSERT_TRUE(first.metrics.settleTime.has_value());
    EXPECT_NEAR(0.0f, *first.metrics.settleTime, 0.02f);
    EXPECT_EQ(1u, benchmark.Current());
    EXPECT_EQ(2u, std::get<arm_host::SelectArmController>(posted.back()).id);
    EXPECT_FLOAT_EQ(15.0f, std::get<arm::BandwidthTuning>(benchmark.Results()[1].tuning).bandwidth);

    Accept();
    Trace(1.1f, 0.0f);
    RunPath();
    Trace(1.1f, 0.02f);

    EXPECT_EQ(BenchmarkState::done, benchmark.State());
    EXPECT_EQ(BenchmarkRunState::done, benchmark.Results()[1].state);
    EXPECT_FALSE(benchmark.Results()[1].metrics.settleTime.has_value());
    EXPECT_EQ(0u, std::get<arm_host::SelectArmController>(posted.back()).id);
}

TEST_F(BenchmarkTest, a_refusal_or_a_trip_fails_the_run_ends_the_benchmark_and_restores_the_controller)
{
    ASSERT_EQ(std::nullopt, benchmark.Start());
    benchmark.OnEvent({ .kind = arm_host::ArmEventKind::refused, .command = 1, .commandName = "select arm controller", .reason = "moving" });

    EXPECT_EQ(BenchmarkState::failed, benchmark.State());
    EXPECT_EQ("select arm controller refused: moving", benchmark.Failure());
    EXPECT_EQ(BenchmarkRunState::failed, benchmark.Results()[0].state);
    EXPECT_EQ(BenchmarkRunState::pending, benchmark.Results()[1].state);
    EXPECT_EQ(0u, std::get<arm_host::SelectArmController>(posted.back()).id);

    tuning::Benchmark tripped{ arm, { ArmControllerKind::gravityPd } };
    ASSERT_EQ(std::nullopt, tripped.Start());
    tripped.OnEvent({ .kind = arm_host::ArmEventKind::tripped, .reason = "J2 tracking error" });
    EXPECT_EQ("tripped: J2 tracking error", tripped.Failure());
    tripped.OnEvent({ .kind = arm_host::ArmEventKind::protectiveStop });
    EXPECT_EQ("tripped: J2 tracking error", tripped.Failure());
}

TEST_F(BenchmarkTest, abort_stops_the_move_and_restores_the_controller)
{
    ASSERT_EQ(std::nullopt, benchmark.Start());
    Accept();
    Trace(1.1f, 0.0f);

    benchmark.Abort();

    EXPECT_EQ(BenchmarkState::aborted, benchmark.State());
    EXPECT_EQ(1u, Count<arm_host::AbortMove>());
    EXPECT_EQ("aborted", benchmark.Results()[0].failure);
    EXPECT_EQ(0u, std::get<arm_host::SelectArmController>(posted.back()).id);
    benchmark.Abort();
    EXPECT_EQ(1u, Count<arm_host::AbortMove>());
}

TEST_F(BenchmarkTest, a_move_aborted_or_a_protective_stop_fails_the_benchmark_with_its_reason)
{
    ASSERT_EQ(std::nullopt, benchmark.Start());
    Accept();
    Trace(1.1f, 0.0f);
    benchmark.OnEvent({ .kind = arm_host::ArmEventKind::moveAborted });
    EXPECT_EQ("move aborted", benchmark.Failure());

    tuning::Benchmark stopped{ arm, { ArmControllerKind::gravityPd } };
    ASSERT_EQ(std::nullopt, stopped.Start());
    stopped.OnEvent({ .kind = arm_host::ArmEventKind::protectiveStop, .reason = "J3 collision" });
    EXPECT_EQ("protective stop: J3 collision", stopped.Failure());
    EXPECT_EQ(BenchmarkState::failed, stopped.State());
}

TEST_F(BenchmarkTest, a_refused_start_posts_nothing)
{
    tuning::Benchmark none{ arm, {} };
    EXPECT_EQ("choose at least one controller", none.Start());
    EXPECT_TRUE(posted.empty());
    EXPECT_EQ(BenchmarkState::idle, none.State());
}

TEST(BenchmarkCsvTest, writes_one_row_per_controller_then_each_runs_samples)
{
    tuning::BenchmarkResult done;
    done.controller = ArmControllerKind::gravityPd;
    done.state = BenchmarkRunState::done;
    done.metrics = { 0.01f, 0.03f, 12.5f, 4.0f, 0.25f };
    done.samples = { { 0.0f, 0.001f }, { 0.01f, 0.002f } };
    tuning::BenchmarkResult failed;
    failed.controller = ArmControllerKind::gravityPid;
    failed.state = BenchmarkRunState::failed;
    failed.failure = "tripped: J2";

    std::ostringstream output;
    tuning::WriteBenchmarkCsv({ done, failed }, output);

    EXPECT_EQ("controller,result,rms_error_rad,peak_error_rad,peak_torque_nm,energy_j,settle_time_s\n"
              "gravity-pd,done,0.01,0.03,12.5,4,0.25\n"
              "gravity-pid,tripped: J2,0,0,0,0,\n"
              "\ncontroller,time_s,error_norm_rad\n"
              "gravity-pd,0,0.001\n"
              "gravity-pd,0.01,0.002\n",
        output.str());
}
