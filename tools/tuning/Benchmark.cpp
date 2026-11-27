#include "tools/tuning/Benchmark.hpp"
#include "core/arm/ArmControllerTuning.hpp"
#include "tools/arm_host/ArmCommand.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string_view>

namespace tuning
{
    namespace
    {
        constexpr std::array<std::array<float, arm::jointCount>, 4> fixedPoses{ {
            { 0.2f, 0.4f, 0.5f, 0.3f, 0.8f, -0.2f },
            { 0.4f, 0.2f, 0.7f, 0.0f, 0.5f, 0.1f },
            { -0.3f, 0.3f, 0.4f, -0.2f, 0.6f, 0.3f },
            { 0.1f, 0.5f, 0.3f, 0.4f, 0.9f, -0.4f },
        } };
        constexpr float limitMargin{ 0.05f };

        arm::JointPositions Within(const arm::ArmConfig& config, const std::array<float, arm::jointCount>& pose)
        {
            arm::JointPositions positions{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            {
                const auto lower = config.positionLimits[joint].lower.Value() + limitMargin;
                const auto upper = config.positionLimits[joint].upper.Value() - limitMargin;
                positions[joint] = foc::Radians{ lower < upper ? std::clamp(pose[joint], lower, upper) : 0.5f * (lower + upper) };
            }
            return positions;
        }

        arm::JointPositions Measured(const arm_host::ArmSnapshot& snapshot)
        {
            arm::JointPositions positions{};
            for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
                positions[joint] = snapshot.readings[joint].position;
            return positions;
        }
    }

    std::vector<arm::JointPositions> BenchmarkPath(const arm::ArmConfig& config, const arm::JointPositions& start)
    {
        std::vector<arm::JointPositions> path;
        if (config.teachPointCount >= 2)
            for (std::size_t point = 0; point != config.teachPointCount; ++point)
                path.push_back(config.teachPoints[point].joints);
        else
            for (const auto& pose : fixedPoses)
                path.push_back(Within(config, pose));
        path.push_back(start);
        return path;
    }

    std::optional<std::string> BenchmarkRefusal(const std::vector<arm::ArmControllerKind>& controllers, const arm_host::ArmSnapshot& snapshot)
    {
        if (controllers.empty())
            return "choose at least one controller";
        if (controllers.size() > maxBenchmarkControllers)
            return "at most eight controllers";
        if (!snapshot.targetReady)
            return "not connected";
        if (snapshot.moving)
            return "the arm is moving";
        if (snapshot.state != arm::ArmState::enabled)
            return "the arm is not enabled";
        for (const auto kind : controllers)
        {
            const std::string_view name{ arm::ArmControllerName(kind) };
            const auto found = std::ranges::find_if(snapshot.armControllers, [name](const arm_host::ArmControllerStatus& controller)
                {
                    return controller.name == name;
                });
            if (found == snapshot.armControllers.end() || !found->enabled)
                return std::string{ name } + " is not offered";
            if (found->passive)
                return std::string{ name } + " tracks no reference";
        }
        return std::nullopt;
    }

    namespace
    {
        std::string Quoted(const std::string& text)
        {
            std::string quoted{ "\"" };
            for (const auto character : text)
            {
                if (character == '"')
                    quoted += '"';
                quoted += character == '\n' ? ' ' : character;
            }
            return quoted + '"';
        }

        class Gains
        {
        public:
            explicit Gains(std::ostream& output)
                : output(output)
            {}

            void operator()(const arm::BandwidthTuning& tuning) const
            {
                output << "bandwidth " << tuning.bandwidth << " rad/s; damping ratio " << tuning.dampingRatio;
            }

            void operator()(const arm::JointImpedanceTuning& tuning) const
            {
                output << "stiffness ";
                if (!tuning.stiffness.has_value())
                    output << "from inertia";
                for (std::size_t joint = 0; tuning.stiffness.has_value() && joint != arm::jointCount; ++joint)
                    output << (joint == 0 ? "" : " ") << (*tuning.stiffness)[joint];
                output << " N·m/rad; damping ratio " << tuning.dampingRatio;
            }

            void operator()(const arm::CartesianImpedanceTuning& tuning) const
            {
                output << "stiffness " << tuning.translationalStiffness << " N/m " << tuning.rotationalStiffness << " N·m/rad; damping ratio " << tuning.dampingRatio;
            }

            void operator()(const arm::OperationalSpaceTuning& tuning) const
            {
                output << "bandwidth " << tuning.bandwidth << " rad/s; damping ratio " << tuning.dampingRatio << "; singularity damping " << tuning.singularityDamping;
            }

            void operator()(const arm::SlotineLiTuning& tuning) const
            {
                output << "lambda " << tuning.lambda << " 1/s; bandwidth " << tuning.bandwidth << " rad/s; adaptation gain " << tuning.adaptationGain;
            }

        private:
            std::ostream& output;
        };
    }

    void WriteBenchmarkCsv(const std::vector<BenchmarkResult>& results, std::ostream& output)
    {
        constexpr std::array<const char*, 4> states{ "pending", "running", "done", "failed" };
        output << "controller,gains,result,rms_error_rad,peak_error_rad,peak_torque_nm,energy_j,settle_time_s\n";
        for (const auto& result : results)
        {
            const auto& metrics = result.metrics;
            output << arm::ArmControllerName(result.controller) << ",\"";
            std::visit(Gains{ output }, result.tuning);
            output << "\"," << Quoted(result.failure.empty() ? states[static_cast<std::size_t>(result.state)] : result.failure) << ',' << metrics.rmsError << ','
                   << metrics.peakError << ',' << metrics.peakTorque << ',' << metrics.energy << ',';
            if (metrics.settleTime.has_value())
                output << *metrics.settleTime;
            output << '\n';
        }
        output << "\ncontroller,time_s,error_norm_rad\n";
        for (const auto& result : results)
            for (const auto& sample : result.samples)
                output << arm::ArmControllerName(result.controller) << ',' << sample.time << ',' << sample.errorNorm << '\n';
    }

    Benchmark::Benchmark(arm_host::ArmCommands& arm, std::vector<arm::ArmControllerKind> controllers)
        : commands(arm)
        , controllers(std::move(controllers))
    {}

    std::optional<std::string> Benchmark::Start()
    {
        if (state == BenchmarkState::running)
            return "a benchmark is running";
        snapshot = commands.Snapshot();
        if (auto refusal = BenchmarkRefusal(controllers, snapshot))
            return refusal;

        failure.clear();
        original.reset();
        pending = 0;
        for (std::size_t id = 0; id != snapshot.armControllers.size(); ++id)
            if (snapshot.armControllers[id].active)
                original = static_cast<arm::ArmControllerId>(id);
        path = BenchmarkPath(snapshot.config, Measured(snapshot));
        results.clear();
        for (const auto kind : controllers)
        {
            BenchmarkResult result;
            result.controller = kind;
            result.tuning = arm::TuningOf(snapshot.config, kind);
            results.push_back(std::move(result));
        }
        state = BenchmarkState::running;
        Begin(0);
        return std::nullopt;
    }

    void Benchmark::Abort()
    {
        if (state != BenchmarkState::running)
            return;
        commands.Post(arm_host::AbortMove{});
        results[current].state = BenchmarkRunState::failed;
        results[current].failure = "aborted";
        state = BenchmarkState::aborted;
        Restore();
    }

    void Benchmark::Begin(std::size_t run)
    {
        current = run;
        leg = 0;
        since.reset();
        previousMeasured.reset();
        squaredSum = 0.0f;
        lastMoveDone = 0.0f;
        lastOutside = 0.0f;
        results[current].state = BenchmarkRunState::running;
        step = Step::selecting;
        pending = commands.Post(arm_host::SelectArmController{ *IdOf(controllers[current]) });
    }

    void Benchmark::OnEvent(const arm_host::ArmEvent& event)
    {
        if (state != BenchmarkState::running)
            return;

        if (pending != 0 && event.command == pending)
        {
            pending = 0;
            if (event.kind == arm_host::ArmEventKind::refused)
                Fail(std::string{ event.commandName } + " refused: " + event.reason);
            else if (event.kind == arm_host::ArmEventKind::accepted && step == Step::selecting)
                step = Step::holding;
        }
        else if (event.kind == arm_host::ArmEventKind::moveAborted)
            Fail("move aborted");
        else if (event.kind == arm_host::ArmEventKind::tripped)
            Fail("tripped: " + event.reason);
        else if (event.kind == arm_host::ArmEventKind::protectiveStop)
            Fail("protective stop: " + event.reason);
        // The arm host runs one move at a time and reports its end without a command id.
        else if (event.kind == arm_host::ArmEventKind::moveDone && step == Step::moving)
            NextLeg();
    }

    void Benchmark::NextLeg()
    {
        if (++leg < path.size())
        {
            pending = commands.Post(arm_host::MoveTo{ path[leg] });
            return;
        }
        step = Step::recording;
        lastMoveDone = results[current].samples.empty() ? 0.0f : results[current].samples.back().time;
    }

    void Benchmark::OnTrace(const arm_host::TraceSample& sample)
    {
        if (state != BenchmarkState::running || step == Step::selecting)
            return;
        if (!since.has_value())
            since = sample.time;
        const auto time = std::chrono::duration<float>(sample.time - *since).count();

        if (step == Step::holding)
        {
            if (time < benchmarkHoldSeconds)
                return;
            since = sample.time;
            step = Step::moving;
            pending = commands.Post(arm_host::MoveTo{ path[0] });
            return;
        }

        Record(sample, time);
        if (state == BenchmarkState::running && step == Step::recording && time - lastMoveDone >= benchmarkHoldSeconds)
            Finish();
    }

    void Benchmark::Record(const arm_host::TraceSample& sample, float time)
    {
        auto& result = results[current];
        if (result.samples.size() == maxBenchmarkSamples)
        {
            Fail("the run exceeded " + std::to_string(maxBenchmarkSamples) + " samples", true);
            return;
        }

        const auto norm = Accumulate(sample, result.metrics);
        bool outside{ false };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
            outside = outside || std::abs(sample.commanded[joint].Value() - sample.measured[joint].Value()) > benchmarkSettleBand;
        previousMeasured = sample.measured;
        squaredSum += norm;
        if (outside)
            lastOutside = time;
        result.samples.push_back({ time, std::sqrt(norm) });
    }

    float Benchmark::Accumulate(const arm_host::TraceSample& sample, BenchmarkMetrics& metrics)
    {
        float norm{ 0.0f };
        for (std::size_t joint = 0; joint != arm::jointCount; ++joint)
        {
            const auto error = std::abs(sample.commanded[joint].Value() - sample.measured[joint].Value());
            norm += error * error;
            metrics.peakError = std::max(metrics.peakError, error);
            if (!sample.torques.has_value())
                continue;
            const auto torque = (*sample.torques)[joint].Value();
            metrics.peakTorque = std::max(metrics.peakTorque, std::abs(torque));
            if (previousMeasured.has_value())
                metrics.energy += std::max(0.0f, torque * (sample.measured[joint].Value() - (*previousMeasured)[joint].Value()));
        }
        return norm;
    }

    void Benchmark::Finish()
    {
        auto& result = results[current];
        result.metrics.rmsError = std::sqrt(squaredSum / static_cast<float>(std::max<std::size_t>(1, result.samples.size())));
        const auto end = result.samples.back().time;
        if (lastOutside < end)
            result.metrics.settleTime = std::max(0.0f, lastOutside - lastMoveDone);
        result.state = BenchmarkRunState::done;

        if (current + 1 < results.size())
            Begin(current + 1);
        else
        {
            state = BenchmarkState::done;
            Restore();
        }
    }

    void Benchmark::Fail(std::string reason, bool stopMove)
    {
        if (stopMove)
            commands.Post(arm_host::AbortMove{});
        results[current].state = BenchmarkRunState::failed;
        results[current].failure = reason;
        failure = std::move(reason);
        state = BenchmarkState::failed;
        Restore();
    }

    void Benchmark::Restore()
    {
        pending = 0;
        if (original.has_value())
            commands.Post(arm_host::SelectArmController{ *original });
    }

    std::optional<arm::ArmControllerId> Benchmark::IdOf(arm::ArmControllerKind kind) const
    {
        const std::string_view name{ arm::ArmControllerName(kind) };
        for (std::size_t id = 0; id != snapshot.armControllers.size(); ++id)
            if (snapshot.armControllers[id].name == name)
                return static_cast<arm::ArmControllerId>(id);
        return std::nullopt;
    }

    BenchmarkState Benchmark::State() const
    {
        return state;
    }

    std::size_t Benchmark::Current() const
    {
        return current;
    }

    const std::vector<BenchmarkResult>& Benchmark::Results() const
    {
        return results;
    }

    const std::string& Benchmark::Failure() const
    {
        return failure;
    }
}
